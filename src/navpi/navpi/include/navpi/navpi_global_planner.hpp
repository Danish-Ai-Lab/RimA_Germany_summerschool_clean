#ifndef NAVPI_GLOBAL_PLANNER_H
#define NAVPI_GLOBAL_PLANNER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "navpi_interfaces/action/plan_path.hpp"
#include "navpi_interfaces/action/recover.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"

#include "astar_vdb/astar_vdb.h"
#include "planner_vdb_base/VDBVisualization.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <navpi_interfaces/action/detail/plan_path__struct.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <navpi/navpi_global_planner_parameters.hpp>
#include <eigen3/Eigen/Geometry>

namespace navpi {

struct AstarVDBPlannerConfig
{
  bool debug_mode           = false;
  bool ground_cache_on      = true;
  int ground_search_level   = 12;
  int successor_variant     = 3;
  int successor_restriction = 2;
  int step_height           = 4;
  int dilate_level          = 5;
  int slope_ground_range    = 3;
  double max_slope_angle    = 45;
};


class NavPiGlobalPlanner : public rclcpp_lifecycle::LifecycleNode
{
  using ValueType          = float;
  using ValueGridT         = openvdb::Grid<typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type>;
  using PlanPath           = navpi_interfaces::action::PlanPath;
  using GoalHandlePlanPath = rclcpp_action::ServerGoalHandle<PlanPath>;
  using Recover            = navpi_interfaces::action::Recover;
  using GoalHandleRecover  = rclcpp_action::ClientGoalHandle<Recover>;

  enum struct State
  {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FINALIZED
  };


public:
  explicit NavPiGlobalPlanner(const rclcpp::NodeOptions& options);
  virtual ~NavPiGlobalPlanner(){};

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  void setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr);

  void makePlan(const geometry_msgs::msg::PoseStamped& start,
                const geometry_msgs::msg::PoseStamped& goal,
                double tolerance,
                std::vector<geometry_msgs::msg::PoseStamped>& path,
                double& cost,
                std::string& message);

  bool cancel();

  bool isInitialized() { return m_state == State::INACTIVE || m_state == State::ACTIVE; }

private:
  geometry_msgs::msg::PoseStamped coordToPose(openvdb::Coord coord);
  openvdb::Coord poseToMapCoord(geometry_msgs::msg::PoseStamped pose);
  geometry_msgs::msg::PoseStamped transformToPose(geometry_msgs::msg::TransformStamped transform);


  void addOrientation(std::vector<geometry_msgs::msg::PoseStamped>& path);

  void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path);

  void publishGridVisualization();
  void publishCostVisualization();
  void publishExpandedVisualization();
  void publishStartAndGoalVisualization();
  void createMarkerMsg(
    visualization_msgs::msg::Marker& marker_msg,
    std::shared_ptr<planner_vdb_base::VDBVisualization<astar_vdb::AStarGridT>::RGBPointCloudT>
      cloud_ptr,
    double voxel_size,
    std::string frame_id);

  void addRGBMarker(visualization_msgs::msg::Marker& marker_msg,
                    const astar_vdb::ValueGridT::Ptr grid,
                    openvdb::Coord index_coord,
                    int r,
                    int g,
                    int b,
                    std::string frame_id);

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const navpi_interfaces::action::PlanPath::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlanPath> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle);

  void executePlanPath(const std::shared_ptr<GoalHandlePlanPath> goal_handle);

  rclcpp_action::Server<PlanPath>::SharedPtr m_plan_path_action_server;


  void recoverGoalResponseCB(const GoalHandleRecover::SharedPtr& goal_handle);
  void recoverFeedbackCB(GoalHandleRecover::SharedPtr,
                         const std::shared_ptr<const Recover::Feedback> feedback);
  void recoverResultCB(const GoalHandleRecover::WrappedResult& result);
  rclcpp_action::Client<Recover>::SharedPtr m_recover_action_client;


  // TODO change for lifecyle stuff
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;

  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;
  bool m_debug_mode = false;
  ValueGridT::Ptr m_vdb_grid;

  std::string m_map_frame;
  std::string m_robot_frame;

  std::shared_ptr<astar_vdb::AstarVDB> m_astar_ptr;
  AstarVDBPlannerConfig m_astar_config;

  std::chrono::time_point<std::chrono::high_resolution_clock> m_timer_start;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_timer_end;

  State m_state = State::UNCONFIGURED;

  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  std::shared_ptr<navpi_global_planner::ParamListener> m_param_listener;
  rclcpp::CallbackGroup::SharedPtr m_plan_path_cb_group;
};

} // namespace navpi
#endif /* NAVPI_GLOBAL_PLANNER_H */
