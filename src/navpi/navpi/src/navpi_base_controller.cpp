#include "navpi/navpi_base_controller.hpp"
#include <tf2/time.h>

namespace navpi {

// NavPiBaseController::NavPiBaseController(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
NavPiBaseController::NavPiBaseController(
  rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<navpi_local_planner::ParamListener> param_listener)
  : m_clock(clock)
  , m_param_listener_ptr(param_listener)
  , m_tf_buffer(clock)
  , m_tf_listener(m_tf_buffer)
{
  m_last_update_time            = clock->now();
  m_params                      = m_param_listener_ptr->get_params();
  m_robot_frame                 = m_params.robot_frame;
  m_dist_tolerance              = m_params.goal_distance_tolerance;
  m_disregard_position_accuracy = m_params.disregard_position_accuracy;
  m_pids.resize(4);
}


geometry_msgs::msg::TwistStamped
NavPiBaseController::updateCommand(const geometry_msgs::msg::PoseStamped& target_pose)
{
  rclcpp::Duration period = m_clock->now() - m_last_update_time;
  m_last_update_time      = m_clock->now();

  geometry_msgs::msg::PoseStamped target_in_robot;
  try
  {
    tf2::doTransform(
      target_pose,
      target_in_robot,
      m_tf_buffer.lookupTransform(m_robot_frame, target_pose.header.frame_id, tf2::TimePointZero));
  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Transform Failed" << std::endl;
  }

  double err_linear_x = target_in_robot.pose.position.x;
  double err_linear_y = target_in_robot.pose.position.y;
  double err_linear_z;

  if (m_params.consider_altitude)
  {
    err_linear_z = target_in_robot.pose.position.z;
  }
  else
  {
    err_linear_z = 0.0;
  }

  double err_linear_all = std::sqrt((err_linear_x * err_linear_x) + (err_linear_y * err_linear_y) +
                                    (err_linear_z * err_linear_z));

  double err_angular_yaw =
    -1 * std::atan2(-1 * target_in_robot.pose.position.y, -1 * target_in_robot.pose.position.x);

  if (err_linear_all < m_dist_tolerance)
  {
    if (m_disregard_position_accuracy)
    {
      err_linear_all = 0;
    }
    tf2::Quaternion q(target_in_robot.pose.orientation.x,
                      target_in_robot.pose.orientation.y,
                      target_in_robot.pose.orientation.z,
                      target_in_robot.pose.orientation.w);
    double r, p;
    tf2::Matrix3x3(q).getRPY(r, p, err_angular_yaw);
  }


  double cmd_x   = m_pids[0]->computeCommand(err_linear_x, (uint64_t)period.nanoseconds());
  double cmd_y   = m_pids[1]->computeCommand(err_linear_y, (uint64_t)period.nanoseconds());
  double cmd_z   = m_pids[2]->computeCommand(err_linear_z, (uint64_t)period.nanoseconds());
  double cmd_yaw = m_pids[3]->computeCommand(err_angular_yaw, (uint64_t)period.nanoseconds());


  geometry_msgs::msg::TwistStamped twist_return;
  twist_return.header.stamp    = m_clock->now();
  twist_return.header.frame_id = m_robot_frame;
  twist_return.twist.linear.x  = cmd_x;
  twist_return.twist.linear.y  = cmd_y;
  twist_return.twist.linear.z  = cmd_z;
  twist_return.twist.angular.z = cmd_yaw;

  // TODO clamp

  return twist_return;
}


void NavPiBaseController::updatePidParams()
{
  std::cout << "Setting new gains" << std::endl;
  for (size_t i = 0; i < 4; ++i)
  {
    const auto& gains = m_params.gains.twist_directions_map.at(m_params.twist_directions[i]);
    std::cout << "gains.p: " << gains.p << std::endl;
    if (m_pids[i])
    {
      // m_pids[i]->setGains(gains.p, gains.i, gains.d, gains.i_max, gains.i_min,
      // gains.anti_windup);
      m_pids[i]->setGains(gains.p, gains.i, gains.d, gains.i_max, gains.i_min, gains.anti_windup);
    }
    else
    {
      m_pids[i] = std::make_shared<control_toolbox::Pid>(
        gains.p, gains.i, gains.d, gains.i_max, gains.i_min, gains.anti_windup);
    }
  }
}

} // namespace navpi
