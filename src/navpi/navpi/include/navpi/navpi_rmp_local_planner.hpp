#ifndef NAVPI_RMP_LOCAL_PLANNER_H
#define NAVPI_RMP_LOCAL_PLANNER_H

#include <cmath>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <mutex>
#include <optional>
#include <rumps/rumps.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navpi_interfaces/action/execute_path.hpp"
#include "std_srvs/srv/empty.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "navpi/navpi_base_controller.hpp"
#include "navpi/navpi_diff_drive_controller.hpp"
#include "navpi/navpi_obstacle_avoidance.hpp"
#include "navpi/navpi_path_sampler.hpp"
#include "navpi/navpi_safety.hpp"

#include <navpi/navpi_rmp_local_planner_parameters.hpp>

namespace navpi {

class NavPiRMPLocalPlanner : public rclcpp_lifecycle::LifecycleNode {
  using ExecutePath = navpi_interfaces::action::ExecutePath;
  using GoalHandleExecutePath = rclcpp_action::ServerGoalHandle<ExecutePath>;

public:
  explicit NavPiRMPLocalPlanner(const rclcpp::NodeOptions &options);
  ~NavPiRMPLocalPlanner() override = default;

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  void setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr);

private:
  /// Pointer to the current VDB map.
  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;

  /// Action server handle for the execute_path
  rclcpp_action::Server<ExecutePath>::SharedPtr
      m_execute_path_action_server_ptr;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_goal_publisher_ptr;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      m_cmd_vel_publisher;

  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr
      m_accel_publisher_ptr;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_marker_publisher;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_occ_grid_publisher;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_force_field_service;

  /// TF listener pointer.
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_ptr{nullptr};

  /// TF buffer pointer.
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_ptr{nullptr};

  /// Thread exceuting the current movement goal.
  std::unique_ptr<std::thread> m_execute_path_thread_ptr;

  /// Current movement goal that is executed.
  std::optional<std::shared_ptr<GoalHandleExecutePath>>
      m_current_goal_handle_ptr;

  /// Next movement goal that should be executed.
  std::optional<std::shared_ptr<GoalHandleExecutePath>> m_next_goal_handle_ptr;

  /// Mutex to secure the goal handles.
  std::mutex m_current_goal_handle_mutex{};

  /// Mutex to secure the goal handles.
  std::mutex m_next_goal_handle_mutex{};

  std::shared_ptr<navpi_rmp_local_planner::ParamListener> m_param_listener_ptr;

  nav_msgs::msg::Path m_path;

  /// Should the current goal be canceled. This should be modified only in the
  /// context of m_goal_handle_mutex
  bool m_should_cancel;

  /// Should shutdown background thread!
  bool m_should_shutdown;

  navpi_rmp_local_planner::Params m_params;

  std::unique_ptr<navpi::NavPiPathSampler> m_path_sampler;
  std::unique_ptr<navpi::NavPiDiffDriveController> m_diff_drive_controller;

  void executePathThread();

  rclcpp_action::GoalResponse
  handleExecutePathGoal(const rclcpp_action::GoalUUID &uuid,
                        std::shared_ptr<const ExecutePath::Goal> &goal);
  rclcpp_action::CancelResponse handleExecutePathCancel(
      const std::shared_ptr<GoalHandleExecutePath> &goal_handle);
  void handleExecutePathAccepted(
      const std::shared_ptr<GoalHandleExecutePath> &goal_handle);
  void executePath(const std::shared_ptr<GoalHandleExecutePath> &goal_handle);
  bool goalIsNotReached(const geometry_msgs::msg::PoseStamped &goal_pose,
                        const geometry_msgs::msg::PoseStamped &current_pose);
  std::shared_ptr<navpi_interfaces::action::ExecutePath::Feedback>
  populateFeedback(const geometry_msgs::msg::PoseStamped &current_pose,
                   const geometry_msgs::msg::PoseStamped &goal_pose);
  void publishAccelMsg(const rumps::SE3::Vec &q_ddot);
  geometry_msgs::msg::PoseStamped tfLookupCurrentPose();
  tf2::Stamped<tf2::Transform>
  poseToTransform(const geometry_msgs::msg::PoseStamped &pose_in);
  rclcpp::CallbackGroup::SharedPtr m_execute_path_cb_group;

  void updatePreTargetFrame(const geometry_msgs::msg::PoseStamped &pre_target);

  void computeConfiguration(const geometry_msgs::msg::PoseStamped &current_pose,
                            rumps::SE3::Vec &q);

  void
  publishForceField(rumps::Root<rumps::SE3>& root,
                    const rumps::SE3::Vec &q,
                    const rumps::SE3::Vec &q_dot = rumps::SE3::Vec::Zero());
  void
  publishForceField(rumps::Root<rumps::SE3>& root,
                    const rumps::SE3::Vec &min, const rumps::SE3::Vec &max,
                    double resolution, double timeout = 0.0,
                    const rumps::SE3::Vec &q_dot = rumps::SE3::Vec::Zero());
  void collectObstacles(std::vector<Eigen::Vector3d> &obstacles,
                        const Eigen::Vector3d &pose);

  void publishForceFieldCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void buildTree(const std::vector<Eigen::Vector3d> &obstacles,
                 const Eigen::Vector3d &goal, rumps::Root<rumps::SE3> &root);

  void handlePathFollowing(const rumps::SE3::Vec& q, rumps::SE3::Vec& q_dot, rumps::SE3::Vec& q_dot_robot, Eigen::Vector3d& goal, double period_ns);

  void handleOrientationAlignment(const rumps::SE3::Vec& q, rumps::SE3::Vec& q_dot, rumps::SE3::Vec& q_dot_robot, Eigen::Matrix3d& target_orientation, double period_ns);
};

} // namespace navpi
#endif /* NAVPI_RMP_LOCAL_PLANNER_H */
