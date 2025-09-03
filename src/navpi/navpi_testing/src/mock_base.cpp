#include "navpi_testing/mock_base.hpp"
#include "navpi_testing/mock_base_parameters.hpp"
#include <functional>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace navpi {

MockBase::MockBase()
    : rclcpp::Node("mock_base")
    , m_twist(TwistVec::Zero())
    , m_pose(TwistVec::Zero()) {
  m_twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      std::bind(&MockBase::twistCallback, this, std::placeholders::_1));

  m_param_listener = std::make_shared<mock_base::ParamListener>(get_node_parameters_interface());
  m_params = m_param_listener->get_params();

  m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  m_last_integration = get_clock()->now();
  m_last_twist = get_clock()->now();

  m_pose << m_params.initial_pose.x,
            m_params.initial_pose.y,
            m_params.initial_pose.z,
            m_params.initial_pose.roll,
            m_params.initial_pose.pitch,
            m_params.initial_pose.yaw;

  using namespace std::chrono_literals;
  m_timer = create_wall_timer(100ms, std::bind(&MockBase::timerCallback, this));
}

void MockBase::twistCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  Eigen::fromMsg(msg->twist, m_twist);
  m_last_twist = get_clock()->now();
}

void MockBase::timerCallback() {
  // publish tf
  publishTransform();

  // perform integration step
  rclcpp::Time now = get_clock()->now();

  double dt = (now - m_last_integration).seconds();
  m_last_integration = now;

  if (dt > m_params.max_dt) {
    RCLCPP_ERROR(get_logger(), "time delta too large for integration step: %f", dt);
    return;
  }

  double twist_age = (now - m_last_twist).seconds();
  if (twist_age > m_params.max_twist_age) {
    RCLCPP_ERROR(get_logger(), "twist is too old, age: %f", twist_age);
    m_twist.setZero();
    return;
  }

  TwistVec global_twist;
  transformTwistToGlobal(global_twist);

  m_pose += global_twist * dt;
}

void MockBase::publishTransform() {
  // publish tf frame
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = get_clock()->now();
  transform.header.frame_id = m_params.map_frame;
  transform.child_frame_id = m_params.robot_frame;

  // Set translation
  transform.transform.translation.x = m_pose(0);
  transform.transform.translation.y = m_pose(1);
  transform.transform.translation.z = m_pose(2);

  // Convert RPY to quaternion
  tf2::Quaternion q;
  q.setRPY(m_pose(3), m_pose(4), m_pose(5)); // roll, pitch, yaw
  q.normalize();

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  m_broadcaster->sendTransform(transform);
}

void MockBase::transformTwistToGlobal(TwistVec& twist) {
  Eigen::AngleAxisd rollAngle(m_pose(3), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(m_pose(4), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(m_pose(5), Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> R = yawAngle * pitchAngle * rollAngle;

  // Transform linear
  twist.head<3>() = R * m_twist.head<3>();
  twist.tail<3>() = R * m_twist.tail<3>();
}
}
