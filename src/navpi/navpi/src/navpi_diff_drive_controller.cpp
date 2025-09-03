#include <navpi/navpi_diff_drive_controller.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/utils.h>

namespace navpi {
NavPiDiffDriveController::NavPiDiffDriveController(
    rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<navpi_diff_drive_controller::ParamListener> param_listener,
    rclcpp::Logger logger)
    : m_param_listener(param_listener), m_logger(logger) {
  m_params = m_param_listener->get_params();
}

void NavPiDiffDriveController::updateCommand(const Vector6d &x_ddot_world, Vector6d &x_dot_world, Vector6d& x_dot_robot, double period_ns) {
  m_params = m_param_listener->get_params();

  m_rotation_pid.setGains(
    m_params.pid.yaw.p,
    m_params.pid.yaw.i,
    m_params.pid.yaw.d,
    m_params.pid.yaw.i_max,
    m_params.pid.yaw.i_min,
    m_params.pid.yaw.anti_windup
  );

  Vector6d unclamped_x_ddot_world = x_dot_world + period_ns * 10e-9 * x_ddot_world;

  auto inverse = m_transform.rotation().inverse();
  x_dot_robot.head<3>() = inverse * unclamped_x_ddot_world.head<3>();
  x_dot_robot.tail<3>() = inverse * unclamped_x_ddot_world.tail<3>();

  if (m_params.rotate_in_movement_direction) {
    double target_yaw = std::atan2(x_dot_robot.y(), x_dot_robot.x());

    if (x_dot_robot(0) < 0.0) {
      if (m_params.only_forward) {
        x_dot_robot(0) = 0.0;
      } else {
        target_yaw = std::fmod(target_yaw + M_PI, (2.0 * M_PI));
      }
    }

    double yaw_rate = m_rotation_pid.computeCommand(target_yaw, period_ns);

    // remove y component
    x_dot_robot(1) = 0.0;

    x_dot_robot(5) = yaw_rate;
  }
  clampTwist(x_dot_robot);

  x_dot_world.head<3>() = m_transform.rotation() * x_dot_robot.head<3>();
  x_dot_world.tail<3>() = m_transform.rotation() * x_dot_robot.tail<3>();
}

void
NavPiDiffDriveController::alignOrientation(Vector6d& x_dot_robot, const Eigen::Vector3d &orientation, double period_ns) {
  m_params = m_param_listener->get_params();

  m_rotation_pid.setGains(
    m_params.pid.yaw.p,
    m_params.pid.yaw.i,
    m_params.pid.yaw.d,
    m_params.pid.yaw.i_max,
    m_params.pid.yaw.i_min,
    m_params.pid.yaw.anti_windup
  );

  // extract current orientation
  Eigen::Vector3d current_orientation = m_transform.rotation().eulerAngles(0, 1, 2);

  // do control
  double orientation_error = (orientation - current_orientation)(2);
  x_dot_robot(5) = m_rotation_pid.computeCommand(orientation_error, period_ns);

  RCLCPP_INFO(m_logger, "Orientation error: %f", orientation_error);

  // set linear velocities to zero
  x_dot_robot.head<3>().setZero();

  clampTwist(x_dot_robot);
}

void NavPiDiffDriveController::setPose(const geometry_msgs::msg::Pose &pose) {
  Eigen::fromMsg(pose, m_transform);
}

void NavPiDiffDriveController::clampTwist(Vector6d& twist) {
  // enforce bounds
  twist <<
    std::clamp(twist(0), -m_params.bounds.linear.x, m_params.bounds.linear.x),
    std::clamp(twist(1), -m_params.bounds.linear.y, m_params.bounds.linear.y),
    std::clamp(twist(2), -m_params.bounds.linear.z, m_params.bounds.linear.z),
    std::clamp(twist(3), -m_params.bounds.angular.x, m_params.bounds.angular.x),
    std::clamp(twist(4), -m_params.bounds.angular.y, m_params.bounds.angular.y),
    std::clamp(twist(5), -m_params.bounds.angular.z, m_params.bounds.angular.z);
}

void NavPiDiffDriveController::resetPID() {
  m_rotation_pid.reset();
}
} // namespace navpi
