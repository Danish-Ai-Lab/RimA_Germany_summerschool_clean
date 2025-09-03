#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <navpi/navpi_diff_drive_controller_parameters.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <control_toolbox/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <Eigen/Eigen>

#include <control_toolbox/pid.hpp>


namespace navpi {

class NavPiDiffDriveController
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  NavPiDiffDriveController(
      rclcpp::Clock::SharedPtr clock,
      std::shared_ptr<navpi_diff_drive_controller::ParamListener>
          param_listener,
      rclcpp::Logger logger);
  NavPiDiffDriveController()                                      = delete;
  NavPiDiffDriveController(NavPiDiffDriveController&&)                 = default;
  NavPiDiffDriveController(const NavPiDiffDriveController&)            = default;
  NavPiDiffDriveController& operator=(NavPiDiffDriveController&&)      = default;
  NavPiDiffDriveController& operator=(const NavPiDiffDriveController&) = default;
  ~NavPiDiffDriveController()                                     = default;


  void
  updateCommand(const Vector6d &x_ddot_world, Vector6d &x_dot_world, Vector6d& x_dot_robot, double period_ns);

  void
  alignOrientation(Vector6d& x_dot_robot, const Eigen::Vector3d &orientation, double period_ns);

  void setPose(const geometry_msgs::msg::Pose& pose);

  void clampTwist(Vector6d& twist);

  void resetPID();

private:
  Vector6d m_pose;

  Eigen::Isometry3d m_transform;

  std::shared_ptr<navpi_diff_drive_controller::ParamListener> m_param_listener;

  navpi_diff_drive_controller::Params m_params;

  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;

  control_toolbox::Pid m_rotation_pid;
};

} // namespace navpi
