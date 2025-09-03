#include "navpi/navpi_global_planner.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__traits.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace navpi {
NavPiGlobalPlanner::NavPiGlobalPlanner(const rclcpp::NodeOptions& options)
  : LifecycleNode("navpi_global_planner", options)
  , m_param_listener(
      std::make_shared<navpi_global_planner::ParamListener>(get_node_parameters_interface()))
{
  m_recover_action_client = rclcpp_action::create_client<Recover>(this, "recover");
}


NavPiGlobalPlanner::CallbackReturn
NavPiGlobalPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto params   = m_param_listener->get_params();
  m_map_frame   = params.map_frame;
  m_robot_frame = params.robot_frame;


  if (m_map_ptr == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Failure to configure global Planner: Map not set.");
    return CallbackReturn::FAILURE;
  }

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_path_pub    = this->create_publisher<nav_msgs::msg::Path>("path", 1);

  m_plan_path_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  using namespace std::placeholders;
  m_plan_path_action_server = rclcpp_action::create_server<PlanPath>(
    this,
    "plan_path",
    std::bind(&NavPiGlobalPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavPiGlobalPlanner::handleCancel, this, std::placeholders::_1),
    std::bind(&NavPiGlobalPlanner::handleAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    m_plan_path_cb_group);

  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiGlobalPlanner::CallbackReturn
NavPiGlobalPlanner::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::ACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiGlobalPlanner::CallbackReturn
NavPiGlobalPlanner::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiGlobalPlanner::CallbackReturn
NavPiGlobalPlanner::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::UNCONFIGURED;
  return CallbackReturn::SUCCESS;
}
NavPiGlobalPlanner::CallbackReturn
NavPiGlobalPlanner::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::FINALIZED;
  return CallbackReturn::SUCCESS;
}

void NavPiGlobalPlanner::setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr)
{
  m_map_ptr = map_ptr;
}

void NavPiGlobalPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal,
                                  double tolerance,
                                  std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                  double& cost,
                                  std::string& message)
{
  auto params = m_param_listener->get_params();


  m_timer_start = std::chrono::high_resolution_clock::now();
  // TODO set debug mode on off from dyn reconfigure
  std::shared_lock map_lock(*m_map_ptr->getMapMutex());
  RCLCPP_INFO(this->get_logger(), "Creating new planner.");
  m_astar_ptr = std::make_shared<astar_vdb::AstarVDB>(m_map_ptr->getGrid(),
                                                      params.debug_mode,
                                                      params.ground_search_level,
                                                      params.ground_cache_on,
                                                      params.successor_variant,
                                                      params.successor_restriction,
                                                      params.step_height,
                                                      params.dilate_level,
                                                      params.slope_ground_range,
                                                      params.max_slope_angle);
  map_lock.unlock();
  RCLCPP_INFO(this->get_logger(), "New Planner created.");

  m_astar_ptr->setStartAndGoal(poseToMapCoord(start), poseToMapCoord(goal));

  openvdb::Coord start_coord = m_astar_ptr->getStart();
  openvdb::Coord goal_coord  = m_astar_ptr->getGoal();

  if (m_debug_mode)
  {
    publishStartAndGoalVisualization();
    publishGridVisualization();
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  planner_vdb_base::SearchState search_state;
  int n_steps = 0;
  do
  {
    n_steps++;
    search_state = m_astar_ptr->step();
    if (m_debug_mode)
    {
      if ((n_steps % 100) == 0)
      {
        publishExpandedVisualization();
      }
    }
  } while (search_state == planner_vdb_base::SearchState::SEARCHING);

  auto t2                                      = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms = t2 - t1;


  std::vector<openvdb::Coord> path_coords;
  if (search_state == planner_vdb_base::SearchState::SUCCESS)
  {
    std::cout << std::endl;
    std::cout << "Search statistics:" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "Computed path to goal in " << ms.count() << "ms" << std::endl;
    m_astar_ptr->printSearchStats();
    path_coords = m_astar_ptr->extractPath();
    nav_msgs::msg::Path global_path;
    global_path.header.frame_id = m_map_frame;
    global_path.header.stamp    = this->now();
    for (auto& p : path_coords)
    {
      plan.push_back(coordToPose(p));
    }

    smoothPath(plan);
    addOrientation(plan);
    plan[plan.size() - 1].pose.orientation = goal.pose.orientation;
    global_path.poses                      = plan;


    if (m_debug_mode)
    {
      publishExpandedVisualization();
      publishCostVisualization();
    }
  }
  else if (search_state == planner_vdb_base::SearchState::FAILURE)
  {
    std::cout << "Not able to find a valid path" << std::endl;
    // TODO Return some failure
  }
  m_timer_end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> path_generation_time = m_timer_end - m_timer_start;
  std::cout << "Overall path generation time: " << path_generation_time.count() << "ms"
            << std::endl;

  // Clean up astar
  m_astar_ptr.reset();
}

bool NavPiGlobalPlanner::cancel()
{
  if (m_debug_mode)
  {
    publishExpandedVisualization();
    publishCostVisualization();
  }
  RCLCPP_INFO(this->get_logger(), "Canceling planner execution.");
  if (m_astar_ptr != nullptr)
  {
    m_astar_ptr->cancelPlanning();
  }
  RCLCPP_INFO(this->get_logger(), "Waiting for planner to shut down...");
  while (m_astar_ptr != nullptr)
    ;
  RCLCPP_INFO(this->get_logger(), "Planner shut down.");


  return true;
}

geometry_msgs::msg::PoseStamped NavPiGlobalPlanner::coordToPose(openvdb::Coord coord)
{
  std::shared_lock map_lock(*m_map_ptr->getMapMutex());
  openvdb::Vec3d world_coord = m_map_ptr->getGrid()->indexToWorld(coord);
  map_lock.unlock();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = m_map_frame;
  // TODO set stamp
  pose.pose.position.x    = world_coord.x();
  pose.pose.position.y    = world_coord.y();
  pose.pose.position.z    = world_coord.z();
  pose.pose.orientation.w = 1.0;
  return pose;
}

openvdb::Coord NavPiGlobalPlanner::poseToMapCoord(geometry_msgs::msg::PoseStamped pose)
{
  openvdb::Vec3d world_coord;

  world_coord.x() = pose.pose.position.x;
  world_coord.y() = pose.pose.position.y;
  world_coord.z() = pose.pose.position.z;
  std::shared_lock map_lock(*m_map_ptr->getMapMutex());
  return openvdb::Coord::floor(m_map_ptr->getGrid()->worldToIndex(world_coord));
}

geometry_msgs::msg::PoseStamped
NavPiGlobalPlanner::transformToPose(geometry_msgs::msg::TransformStamped transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header           = transform.header;
  pose.pose.position.x  = transform.transform.translation.x;
  pose.pose.position.y  = transform.transform.translation.y;
  pose.pose.position.z  = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

void NavPiGlobalPlanner::addOrientation(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  Eigen::Matrix<double, 3, 1> a, b;
  a << 1, 0, 0;
  Eigen::Quaterniond q;

  for (size_t i = 0; i < path.size() - 1; ++i)
  {
    // Since most platforms just depend on yaw orientation we ignore all other rotations by
    // ignoreing the z coordinate
    b << (path[i + 1].pose.position.x - path[i].pose.position.x),
      (path[i + 1].pose.position.y - path[i].pose.position.y), 0.0;
    q = Eigen::Quaterniond().setFromTwoVectors(a, b);
    q.normalize();

    path[i].pose.orientation.x = q.x();
    path[i].pose.orientation.y = q.y();
    path[i].pose.orientation.z = q.z();
    path[i].pose.orientation.w = q.w();
  }
}

void NavPiGlobalPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  double weight_data   = 0.5;
  double weight_smooth = 0.35;
  double tolerance     = 0.000001;
  std::vector<Eigen::Matrix<double, 3, 1>> input, output;

  for (auto& p : path)
  {
    Eigen::Matrix<double, 3, 1> buffer;
    buffer[0] = p.pose.position.x;
    buffer[1] = p.pose.position.y;
    buffer[2] = p.pose.position.z;
    input.push_back(buffer);
    output.push_back(buffer);
  }

  double change = tolerance;

  while (change >= tolerance)
  {
    change = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i)
    {
      Eigen::Matrix<double, 3, 1> aux = output[i];
      output[i] += weight_data * (input[i] - output[i]);
      output[i] += weight_smooth * (output[i - 1] + output[i + 1] - (2.0 * output[i]));
      change += (aux - output[i]).norm();
    }
  }

  for (size_t i = 0; i < output.size(); ++i)
  {
    path[i].pose.position.x = output[i][0];
    path[i].pose.position.y = output[i][1];
    path[i].pose.position.z = output[i][2];
  }
}

void NavPiGlobalPlanner::publishGridVisualization() {}
void NavPiGlobalPlanner::publishCostVisualization() {}
void NavPiGlobalPlanner::publishExpandedVisualization() {}
void NavPiGlobalPlanner::publishStartAndGoalVisualization() {}
void NavPiGlobalPlanner::createMarkerMsg(
  visualization_msgs::msg::Marker& marker_msg,
  std::shared_ptr<planner_vdb_base::VDBVisualization<astar_vdb::AStarGridT>::RGBPointCloudT>
    cloud_ptr,
  double voxel_size,
  std::string frame_id)
{
}

void NavPiGlobalPlanner::addRGBMarker(visualization_msgs::msg::Marker& marker_msg,
                                      const astar_vdb::ValueGridT::Ptr grid,
                                      openvdb::Coord index_coord,
                                      int r,
                                      int g,
                                      int b,
                                      std::string frame_id)
{
}


rclcpp_action::GoalResponse
NavPiGlobalPlanner::handleGoal(const rclcpp_action::GoalUUID& uuid,
                               std::shared_ptr<const navpi_interfaces::action::PlanPath::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "Received goal requested");
  if (m_state != State::ACTIVE)
  {
    RCLCPP_WARN(this->get_logger(), "Planner not active. Goal rejected.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (m_astar_ptr != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "There is still a running planner. Cleaning up...");
    cancel();
    RCLCPP_INFO(this->get_logger(), "Cleaning up done.");
  }

  // TODO check if there is a case to reject the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
NavPiGlobalPlanner::handleCancel(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  if (m_astar_ptr != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "There is still a running planner. Cleaning up...");
    cancel();
    RCLCPP_INFO(this->get_logger(), "Cleaning up done.");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPiGlobalPlanner::handleAccepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
  std::thread{std::bind(&NavPiGlobalPlanner::executePlanPath, this, std::placeholders::_1),
              goal_handle}
    .detach();
}

void NavPiGlobalPlanner::executePlanPath(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), " execute plan path");
  auto goal   = goal_handle->get_goal();
  auto result = std::make_shared<PlanPath::Result>();

  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
  geometry_msgs::msg::PoseStamped start_in_map;
  geometry_msgs::msg::PoseStamped goal_in_map;

  goal_pose = goal->target_pose;

  if (goal->use_start_pose)
  {
    start_pose = goal->start_pose;
  }
  else
  {
    try
    {
      start_pose = transformToPose(
        m_tf_buffer->lookupTransform(m_map_frame, m_robot_frame, tf2::TimePointZero));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Could not transform %s to %s : %s",
                  m_map_frame.c_str(),
                  m_robot_frame.c_str(),
                  ex.what());
      return;
    }
  }

  if (goal_pose.header.frame_id != m_map_frame)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Goal is given in " << goal_pose.header.frame_id
                                           << ". Transforming it to map frame: " << m_map_frame);

    geometry_msgs::msg::TransformStamped goal_to_map_tf;
    try
    {
      start_pose.header.stamp = this->now();
      goal_pose.header.stamp  = this->now();
      m_tf_buffer->transform(goal_pose, goal_in_map, m_map_frame, tf2::durationFromSec(5.0));
      m_tf_buffer->transform(start_pose, start_in_map, m_map_frame, tf2::durationFromSec(5.0));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Could not transform %s to %s : %s",
                  goal_pose.header.frame_id.c_str(),
                  m_map_frame.c_str(),
                  ex.what());
      // TODO error handling
      return;
    }
  }
  else
  {
    start_in_map = start_pose;
    goal_in_map  = goal_pose;
  }

  auto params = m_param_listener->get_params();

  int retries          = params.global_planner_retries;
  int recovery_retries = params.recover_at_n_tries;
  bool plan_successful = false;

  std::vector<geometry_msgs::msg::PoseStamped> plan;
  double cost;
  std::string message;
  using namespace std::placeholders;
  for (int i = 0; !plan_successful && i < retries; ++i)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Starting planning process. " << i + 1 << " of " << retries << " tries");
    if ((i + 1) > recovery_retries)
    {
      auto recover_goal              = Recover::Goal();
      auto send_recover_goal_options = rclcpp_action::Client<Recover>::SendGoalOptions();
      send_recover_goal_options.goal_response_callback =
        std::bind(&NavPiGlobalPlanner::recoverGoalResponseCB, this, _1);
      send_recover_goal_options.feedback_callback =
        std::bind(&NavPiGlobalPlanner::recoverFeedbackCB, this, _1, _2);
      send_recover_goal_options.result_callback =
        std::bind(&NavPiGlobalPlanner::recoverResultCB, this, _1);

      RCLCPP_INFO(this->get_logger(), "Starting recovery process.");
      auto recover_goal_handle =
        m_recover_action_client->async_send_goal(recover_goal, send_recover_goal_options);


      auto timeout = std::chrono::milliseconds((int)(1000));
      auto recover_result_future =
        m_recover_action_client->async_get_result(recover_goal_handle.get());
      if (recover_result_future.wait_for(timeout) != std::future_status::ready)
      {
        continue;
      }

      auto recover_result_handle = recover_result_future.get();
      if (recover_result_handle.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Recovery executed");
    }


    makePlan(start_in_map, goal_in_map, 0.0, plan, cost, message);

    if (plan.size() > 0)
    {
      plan_successful = true;
    }
  }
  if (!plan_successful)
  {
    result->success                = false;
    result->planner_result.id      = navpi_interfaces::msg::Result::GLOBAL_NO_PATH_FOUND;
    result->planner_result.message = message;
    goal_handle->succeed(result);

    RCLCPP_WARN(this->get_logger(), "Planner found empty path!");
    return;
  }
  else
  {
    nav_msgs::msg::Path global_path;
    global_path.header = start_pose.header;
    global_path.poses  = plan;
    m_path_pub->publish(global_path);

    result->success                = true;
    result->planner_result.id      = navpi_interfaces::msg::Result::SUCCESS;
    result->planner_result.message = message;
    result->path                   = global_path;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Planner execution done.");
  }
}

void NavPiGlobalPlanner::recoverGoalResponseCB(const GoalHandleRecover::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal to recover was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal to recover was accepted by server, waiting for result");
  }
}
void NavPiGlobalPlanner::recoverFeedbackCB(GoalHandleRecover::SharedPtr,
                                           const std::shared_ptr<const Recover::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "recovery feedback");
}
void NavPiGlobalPlanner::recoverResultCB(const GoalHandleRecover::WrappedResult& result) {}

} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiGlobalPlanner)
