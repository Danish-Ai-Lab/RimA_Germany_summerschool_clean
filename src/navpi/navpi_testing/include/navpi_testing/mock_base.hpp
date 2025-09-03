#pragma once

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <navpi_testing/mock_base_parameters.hpp>

namespace navpi {

class MockBase : public rclcpp::Node {
public:
  using TwistVec = Eigen::Matrix<double, 6, 1>;

  MockBase();
  ~MockBase() = default;

  void twistCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void timerCallback();

  void publishTransform();

private:
  void transformTwistToGlobal(TwistVec& twist);

  std::shared_ptr<mock_base::ParamListener> m_param_listener;
  mock_base::Params m_params;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_twist_sub;
  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Time m_last_integration;
  rclcpp::Time m_last_twist;

  TwistVec m_twist;
  TwistVec m_pose;
};

} // namespace navpi
