#ifndef NAVPI_LOCAL_PLANNER_H
#define NAVPI_LOCAL_PLANNER_H

#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navpi_interfaces/action/execute_path.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


#include "navpi/navpi_base_controller.hpp"
#include "navpi/navpi_obstacle_avoidance.hpp"
#include "navpi/navpi_path_sampler.hpp"
#include "navpi/navpi_safety.hpp"


#include <navpi/navpi_local_planner_parameters.hpp>
#include <navpi/navpi_path_sampler_parameters.hpp>

namespace navpi {

class NavPiLocalPlanner : public rclcpp_lifecycle::LifecycleNode
{
  using ExecutePath           = navpi_interfaces::action::ExecutePath;
  using GoalHandleExecutePath = rclcpp_action::ServerGoalHandle<ExecutePath>;

public:
  explicit NavPiLocalPlanner(const rclcpp::NodeOptions& options);
  ~NavPiLocalPlanner() override = default;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  void setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr);


private:
  /// Pointer to the current VDB map.
  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;

  /// Action server handle for the execute_path
  rclcpp_action::Server<ExecutePath>::SharedPtr m_execute_path_action_server_ptr;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_publisher_ptr;


  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel_publisher;

  /// TF listener pointer.
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_ptr{nullptr};

  /// TF buffer pointer.
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_ptr{nullptr};

  /// Thread exceuting the current movement goal.
  std::unique_ptr<std::thread> m_execute_path_thread_ptr;

  /// Current movement goal that is executed.
  std::optional<std::shared_ptr<GoalHandleExecutePath>> m_current_goal_handle_ptr;

  /// Next movement goal that should be executed.
  std::optional<std::shared_ptr<GoalHandleExecutePath>> m_next_goal_handle_ptr;

  /// Mutex to secure the goal handles.
  std::mutex m_current_goal_handle_mutex{};

  /// Mutex to secure the goal handles.
  std::mutex m_next_goal_handle_mutex{};

  std::shared_ptr<navpi_local_planner::ParamListener> m_param_listener_ptr;

  nav_msgs::msg::Path m_path;

  /// Should the current goal be canceled. This should be modified only in the context of
  /// m_goal_handle_mutex
  bool m_should_cancel;

  /// Should shutdown background thread!
  bool m_should_shutdown;

  double m_loop_rate;
  double m_distance_tolerance;
  double m_angle_tolerance;
  std::string m_map_frame;
  std::string m_robot_frame;
  bool m_sampler_only_mode;

  void executePathThread();

  std::unique_ptr<navpi::NavPiPathSampler> m_path_sampler;
  std::unique_ptr<navpi::NavPiBaseController> m_base_controller;
  std::unique_ptr<navpi::NavPiObstacleAvoidance> m_obstacle_avoidance;
  std::unique_ptr<navpi::NavPiSafety> m_safety;


  rclcpp_action::GoalResponse handleExecutePathGoal(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const ExecutePath::Goal>& goal);
  rclcpp_action::CancelResponse
  handleExecutePathCancel(const std::shared_ptr<GoalHandleExecutePath>& goal_handle);
  void handleExecutePathAccepted(const std::shared_ptr<GoalHandleExecutePath>& goal_handle);
  void executePath(const std::shared_ptr<GoalHandleExecutePath>& goal_handle);
  bool goalIsNotReached(const geometry_msgs::msg::PoseStamped& goal_pose,
                        const geometry_msgs::msg::PoseStamped& current_pose);
  std::shared_ptr<navpi_interfaces::action::ExecutePath::Feedback>
  populateFeedback(const geometry_msgs::msg::PoseStamped& current_pose,
                   const geometry_msgs::msg::PoseStamped& goal_pose);
  geometry_msgs::msg::PoseStamped tfLookupCurrentPose();
  tf2::Stamped<tf2::Transform> poseToTransform(const geometry_msgs::msg::PoseStamped& pose_in);
  rclcpp::CallbackGroup::SharedPtr m_execute_path_cb_group;
};

} // namespace navpi
#endif /* NAVPI_LOCAL_PLANNER_H */
