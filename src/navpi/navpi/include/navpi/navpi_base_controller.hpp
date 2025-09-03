#ifndef NAVPI_BASE_CONTROLLER_H
#define NAVPI_BASE_CONTROLLER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <navpi/navpi_local_planner_parameters.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <control_toolbox/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>


namespace navpi {

class NavPiBaseController
{
public:
  NavPiBaseController(rclcpp::Clock::SharedPtr clock,
                      std::shared_ptr<navpi_local_planner::ParamListener> param_listener);
  NavPiBaseController()                                      = delete;
  NavPiBaseController(NavPiBaseController&&)                 = default;
  NavPiBaseController(const NavPiBaseController&)            = default;
  NavPiBaseController& operator=(NavPiBaseController&&)      = default;
  NavPiBaseController& operator=(const NavPiBaseController&) = default;
  ~NavPiBaseController()                                     = default;


  geometry_msgs::msg::TwistStamped
  updateCommand(const geometry_msgs::msg::PoseStamped& target_pose);
  void clampTwist(geometry_msgs::msg::TwistStamped& twist_to_clamp /*, duration */);
  void clampLinearDimension(double& value_to_clamp, double last_odom_value /*, period */);
  void clampAngularDimension(double& value_to_clamp, double last_odom_value /*, period */);

  void updatePidParams();


private:
  double m_min_vel_linear;
  double m_max_vel_linear;

  nav_msgs::msg::Odometry m_last_odom;

  rclcpp_lifecycle::LifecycleNode::SharedPtr m_node;

  std::shared_ptr<navpi_local_planner::ParamListener> m_param_listener_ptr;

  std::vector<std::shared_ptr<control_toolbox::Pid>> m_pids;

  rclcpp::Time m_last_update_time;

  navpi_local_planner::Params m_params;

  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  std::string m_robot_frame;

  rclcpp::Clock::SharedPtr m_clock;
  double m_dist_tolerance;
  bool m_disregard_position_accuracy;
};

} // namespace navpi


#endif // !NAVPI_BASE_CONTROLLER_H
