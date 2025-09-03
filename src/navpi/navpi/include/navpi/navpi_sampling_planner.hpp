#ifndef NAVPI_SAMPLING_PLANNER_H
#define NAVPI_SAMPLING_PLANNER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "navpi_interfaces/action/plan_path.hpp"
#include "navpi_interfaces/action/recover.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include "vdb_mapping_ros2/VDBMappingTools.hpp"


#include "rclcpp_action/rclcpp_action.hpp"
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

#include <navpi/IntegerMotionValidator.h>
#include <navpi/IntegerStateSampler.h>
#include <navpi/IntegerStateSpace.h>
#include <navpi/MapHandle.h>

#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <navpi/RRTstarConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace navpi {

class NavPiSamplingPlanner : public rclcpp_lifecycle::LifecycleNode
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
  explicit NavPiSamplingPlanner(const rclcpp::NodeOptions& options);
  virtual ~NavPiSamplingPlanner(){};

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
                navpi_interfaces::msg::Result& solver_result);

  bool cancel();

  bool isInitialized() { return m_state == State::INACTIVE || m_state == State::ACTIVE; }

  void publishDebugVisualization();
  void publishStartAndGoal(openvdb::Coord start, openvdb::Coord goal);

private:
  geometry_msgs::msg::PoseStamped coordToPose(openvdb::Coord coord);
  openvdb::Coord poseToMapCoord(geometry_msgs::msg::PoseStamped pose);
  geometry_msgs::msg::PoseStamped transformToPose(geometry_msgs::msg::TransformStamped transform);


  geometry_msgs::msg::PoseStamped eigenToPoseStamped(const Eigen::Matrix<double, 3, 1>& P);

  void normalizePath(std::vector<geometry_msgs::msg::PoseStamped>& path);

  void addOrientation(std::vector<geometry_msgs::msg::PoseStamped>& path,
                      geometry_msgs::msg::PoseStamped goal);

  void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path);

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

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_c_space_grid_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_value_grid_pub;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_start_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_goal_marker_pub;

  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;

  std::string m_map_frame;
  std::string m_robot_frame;
  double m_distance_tolerance;
  double m_min_planner_distance;
  bool m_interpolate_path;
  bool m_debug_mode;
  bool m_smooth_path;

  State m_state = State::UNCONFIGURED;

  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  std::shared_ptr<navpi_global_planner::ParamListener> m_param_listener;
  std::shared_ptr<MapHandle> m_map_handle;
  std::string name;
  bool isStateValid(const ob::State* state);
  int m_used_nodes;
  bool m_cancel_requested;

  std::vector<openvdb::Coord> m_goals;
  std::vector<double> m_planning_times;
  rclcpp::CallbackGroup::SharedPtr m_plan_path_cb_group;

  openvdb::FloatGrid::Ptr m_valid_sample_grid;
  openvdb::FloatGrid::Ptr m_invalid_sample_grid;
  std::shared_ptr<openvdb::FloatGrid::Accessor> m_valid_sample_acc;
  std::shared_ptr<openvdb::FloatGrid::Accessor> m_invalid_sample_acc;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_valid_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_invalid_pub;
};

} // namespace navpi
#endif /* NAVPI_SAMPLING_PLANNER_H */
