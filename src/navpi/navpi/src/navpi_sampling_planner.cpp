#include "navpi/navpi_sampling_planner.hpp"

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
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace navpi {
NavPiSamplingPlanner::NavPiSamplingPlanner(const rclcpp::NodeOptions& options)
  : LifecycleNode("navpi_sampling_planner", options)
  , m_param_listener(
      std::make_shared<navpi_global_planner::ParamListener>(get_node_parameters_interface()))
{
  m_recover_action_client = rclcpp_action::create_client<Recover>(this, "recover");
}

NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto params            = m_param_listener->get_params();
  m_map_frame            = params.map_frame;
  m_robot_frame          = params.robot_frame;
  m_distance_tolerance   = params.distance_tolerance;
  m_min_planner_distance = params.min_planner_distance;
  m_debug_mode           = params.debug_mode;
  m_interpolate_path     = params.interpolate_path;
  m_smooth_path          = params.smooth_path;


  if (m_map_ptr == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Failure to configure sampling Planner: Map not set.");
    return CallbackReturn::FAILURE;
  }

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_path_pub    = this->create_publisher<nav_msgs::msg::Path>("path", 1);


  m_start_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("start_maker", 1);
  m_goal_marker_pub  = this->create_publisher<visualization_msgs::msg::Marker>("goal_maker", 1);

  if (m_debug_mode)
  {
    m_c_space_grid_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("c_space_grid", 1);
    m_value_grid_pub   = this->create_publisher<sensor_msgs::msg::PointCloud2>("value_grid", 1);
    m_valid_pub        = this->create_publisher<sensor_msgs::msg::PointCloud2>("valid_samples", 1);
    m_invalid_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("invalid_samples", 1);
  }

  m_plan_path_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  using namespace std::placeholders;
  m_plan_path_action_server = rclcpp_action::create_server<PlanPath>(
    this,
    "plan_path",
    std::bind(
      &NavPiSamplingPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavPiSamplingPlanner::handleCancel, this, std::placeholders::_1),
    std::bind(&NavPiSamplingPlanner::handleAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    m_plan_path_cb_group);

  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::ACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::UNCONFIGURED;
  return CallbackReturn::SUCCESS;
}
NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::FINALIZED;
  return CallbackReturn::SUCCESS;
}

void NavPiSamplingPlanner::setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr)
{
  m_map_ptr = map_ptr;
}

void NavPiSamplingPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                    const geometry_msgs::msg::PoseStamped& goal,
                                    double tolerance,
                                    std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    double& cost,
                                    navpi_interfaces::msg::Result& solver_result)
{
  if (m_debug_mode)
  {
    m_valid_sample_grid = openvdb::FloatGrid::create(0.0);
    m_valid_sample_grid->setTransform(openvdb::math::Transform::createLinearTransform(0.05));
    m_valid_sample_acc =
      std::make_shared<openvdb::FloatGrid::Accessor>(m_valid_sample_grid->getAccessor());
    m_invalid_sample_grid = openvdb::FloatGrid::create(0.0);
    m_invalid_sample_grid->setTransform(openvdb::math::Transform::createLinearTransform(0.05));
    m_invalid_sample_acc =
      std::make_shared<openvdb::FloatGrid::Accessor>(m_invalid_sample_grid->getAccessor());
  }

  double initial_distance = std::sqrt(std::pow((goal.pose.position.x - start.pose.position.x), 2) +
                                      std::pow((goal.pose.position.y - start.pose.position.y), 2) +
                                      std::pow((goal.pose.position.z - start.pose.position.z), 2));

  if (initial_distance < m_min_planner_distance)
  {
    solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_EXACT_SOLUTION;
    solver_result.message = "Short distance solution found";
    std::cout << solver_result.message << std::endl;
    plan.push_back(start);
    plan.push_back(goal);
    return;
  }

  auto setup_t1 = std::chrono::high_resolution_clock::now();

  auto params = m_param_listener->get_params();
  std::shared_lock map_lock(*m_map_ptr->getMapMutex());

  m_map_handle = std::make_shared<MapHandle>(m_map_ptr->getGrid(),
                                             params.dilate_level,
                                             params.closing_level,
                                             params.ground_search_level,
                                             params.max_slope_angle,
                                             params.step_height);
  map_lock.unlock();
  auto bbox = m_map_handle->getValueGrid()->evalActiveVoxelBoundingBox();
  auto space(std::make_shared<IntegerStateSpace>(bbox));
  ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
  si->setStateValidityChecker(
    [this](const ob::State* state) { return NavPiSamplingPlanner::isStateValid(state); });

  auto ompl_setup = std::make_shared<og::SimpleSetup>(si);

  name = "rrt_connect";

  if (name == "rrt")
  {
    ompl_setup->setPlanner(std::make_shared<og::RRT>(ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_connect")
  {
    ompl_setup->setPlanner(std::make_shared<og::RRTConnect>(ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_star")
  {
    ompl_setup->setPlanner(std::make_shared<og::RRTstar>(ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_star_connect")
  {
    ompl_setup->setPlanner(std::make_shared<og::RRTstarConnect>(ompl_setup->getSpaceInformation()));
  }
  else if (name == "prm")
  {
    ompl_setup->setPlanner(std::make_shared<og::PRM>(ompl_setup->getSpaceInformation()));
  }
  else
  {
    ompl_setup->setPlanner(std::make_shared<og::RRTConnect>(ompl_setup->getSpaceInformation()));
  }

  space->setStateSamplerAllocator([&](const ob::StateSpace* ss) {
    return std::make_shared<IntegerStateSampler>(ss, m_map_handle);
  });

  space->setup();
  auto setup_t2 = std::chrono::high_resolution_clock::now();

  openvdb::Coord start_index =
    openvdb::math::Coord::round(m_map_handle->getValueGrid()->worldToIndex(
      openvdb::Vec3d(start.pose.position.x, start.pose.position.y, start.pose.position.z)));
  std::cout << "Start Index initial: " << start_index << std::endl;

  if (m_map_handle->generateValidNode(start_index))
  {
    std::cout << "Valid start index found at: " << start_index << std::endl;
  }
  else
  {
    std::cout << "No valid start index found below initial start. Searching for closest Map index."
              << std::endl;
    start_index = m_map_handle->findClosestMapIndex(start_index);
    if (m_map_handle->generateValidNode(start_index))
    {
      std::cout << "Start Index adapted: " << start_index << std::endl;
    }
    else
    {
      std::cout << "Could not generate a valid start point" << std::endl;
      solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_INVALID_START;
      solver_result.message = "Could not generate a valid start.";
      std::cout << solver_result.message << std::endl;
      return;
    }
  }

  ob::ScopedState<> ompl_start(ompl_setup->getStateSpace());
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value =
    start_index.x();
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value =
    start_index.y();
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value =
    start_index.z();

  openvdb::Coord goal_index =
    openvdb::math::Coord::round(m_map_handle->getValueGrid()->worldToIndex(
      openvdb::Vec3d(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)));
  std::cout << "Goal index initial: " << goal_index << std::endl;

  if (m_map_handle->generateValidNode(goal_index))
  {
    std::cout << "Valid goal index found at: " << goal_index << std::endl;
  }
  else
  {
    std::cout << "No valid goal index found below initial goal. Searching for closest Map index."
              << std::endl;
    goal_index = m_map_handle->findClosestMapIndex(goal_index);

    if (m_map_handle->generateValidNode(goal_index))
    {
      std::cout << "Goal Index adapted: " << goal_index << std::endl;
    }
    else
    {
      std::cout << "Could not generate a valid goal point" << std::endl;
      solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_INVALID_GOAL;
      solver_result.message = "Could not generate a valid goal.";
      std::cout << solver_result.message << std::endl;
      return;
    }
  }


  publishStartAndGoal(start_index, goal_index);

  auto solve_t1 = std::chrono::high_resolution_clock::now();

  ob::ScopedState<> ompl_goal(ompl_setup->getStateSpace());
  ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value =
    goal_index.x();
  ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value =
    goal_index.y();
  ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value =
    goal_index.z();

  ompl_setup->setStartAndGoalStates(ompl_start, ompl_goal);

  int runs;
  double time;
  bool star = name.find("star") != std::string::npos;
  if (star)
  {
    runs = 1;
    time = 5.0;
  }
  else
  {
    runs = 10;
    time = 1.0;
  }

  std::cout << "Starting to solve the Problem" << std::endl;
  for (int i = 0; i < runs; i++)
  {
    if (m_cancel_requested)
    {
      m_cancel_requested = false;
      return;
    }
    ompl_setup->solve(time);
  }
  std::cout << "Done solving the Problem" << std::endl;

  auto solve_t2                                      = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> solve_ms = solve_t2 - solve_t1;

  std::size_t ns = ompl_setup->getProblemDefinition()->getSolutionCount();
  OMPL_INFORM("Found %d solutions", (int)ns);

  if (m_debug_mode)
  {
    publishDebugVisualization();
  }

  if (ompl_setup->haveExactSolutionPath())
  {
    og::PathGeometric& p = ompl_setup->getSolutionPath();
    if (p.check())
    {
      std::cout << "Path is valid" << std::endl;
    }
    else
    {
      std::cout << "Path is invalid" << std::endl;
    }
    std::cout << "Path Length: " << p.length() << std::endl;

    ompl_setup->getPathSimplifier()->simplifyMax(p);

    geometry_msgs::msg::PoseStamped pose_buffer;
    pose_buffer.header.frame_id = m_map_frame;
    // pose_buffer.header.stamp       = ros::Time::now();
    pose_buffer.pose.orientation.w = 1;
    std::cout << "State count: " << p.getStateCount() << std::endl;

    plan.push_back(start);

    for (size_t i = 0; i < p.getStateCount(); i++)
    {
      openvdb::Coord index_pose;
      index_pose.x() = p.getState(i)
                         ->as<IntegerStateSpace::StateType>()
                         ->as<ob::DiscreteStateSpace::StateType>(0)
                         ->value;
      index_pose.y() = p.getState(i)
                         ->as<IntegerStateSpace::StateType>()
                         ->as<ob::DiscreteStateSpace::StateType>(1)
                         ->value;
      index_pose.z() = p.getState(i)
                         ->as<IntegerStateSpace::StateType>()
                         ->as<ob::DiscreteStateSpace::StateType>(2)
                         ->value;

      openvdb::Vec3d world_pose   = m_map_handle->getValueGrid()->indexToWorld(index_pose);
      pose_buffer.pose.position.x = world_pose.x();
      pose_buffer.pose.position.y = world_pose.y();
      pose_buffer.pose.position.z = world_pose.z();
      plan.push_back(pose_buffer);
    }

    double distance_to_goal =
      std::sqrt(std::pow((goal.pose.position.x - pose_buffer.pose.position.x), 2) +
                std::pow((goal.pose.position.y - pose_buffer.pose.position.y), 2) +
                std::pow((goal.pose.position.z - pose_buffer.pose.position.z), 2));

    std::cout << "Position to actual goal is: " << distance_to_goal << std::endl;

    if (m_interpolate_path)
    {
      normalizePath(plan);
    }
    if (m_smooth_path)
    {
      smoothPath(plan);
    }

    if (distance_to_goal > m_distance_tolerance)
    {
      solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_APPROX_SOLUTION;
      solver_result.message = "Approximate solution found.";
      std::cout << solver_result.message << std::endl;
      plan.push_back(goal);
    }
    else
    {
      solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_EXACT_SOLUTION;
      solver_result.message = "Exact solution found.";
      std::cout << solver_result.message << std::endl;
    }
    addOrientation(plan, goal);
  }
  else
  {
    std::cout << " No solution path found" << std::endl;
    solver_result.id      = navpi_interfaces::msg::Result::GLOBAL_NO_PATH_FOUND;
    solver_result.message = "No path found.";
  }
}

bool NavPiSamplingPlanner::cancel()
{
  RCLCPP_INFO(this->get_logger(), "Canceling planner execution.");

  m_cancel_requested = true;
  return true;
}

geometry_msgs::msg::PoseStamped NavPiSamplingPlanner::coordToPose(openvdb::Coord coord)
{
  openvdb::Vec3d world_coord = m_map_handle->getValueGrid()->indexToWorld(coord);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = m_map_frame;
  // TODO set stamp
  pose.pose.position.x    = world_coord.x();
  pose.pose.position.y    = world_coord.y();
  pose.pose.position.z    = world_coord.z();
  pose.pose.orientation.w = 1.0;
  return pose;
}

openvdb::Coord NavPiSamplingPlanner::poseToMapCoord(geometry_msgs::msg::PoseStamped pose)
{
  openvdb::Vec3d world_coord;

  world_coord.x() = pose.pose.position.x;
  world_coord.y() = pose.pose.position.y;
  world_coord.z() = pose.pose.position.z;

  return openvdb::Coord::floor(m_map_handle->getValueGrid()->worldToIndex(world_coord));
}

geometry_msgs::msg::PoseStamped
NavPiSamplingPlanner::transformToPose(geometry_msgs::msg::TransformStamped transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header           = transform.header;
  pose.pose.position.x  = transform.transform.translation.x;
  pose.pose.position.y  = transform.transform.translation.y;
  pose.pose.position.z  = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

// TODO exchange hard coded frame_id
geometry_msgs::msg::PoseStamped
NavPiSamplingPlanner::eigenToPoseStamped(const Eigen::Matrix<double, 3, 1>& P)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = m_map_frame;
  pose.pose.position.x    = P.x();
  pose.pose.position.y    = P.y();
  pose.pose.position.z    = P.z();
  pose.pose.orientation.w = 1.0;
  return pose;
}

void NavPiSamplingPlanner::normalizePath(std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  double distance = 0.05;
  std::vector<geometry_msgs::msg::PoseStamped> old_plan(plan);
  plan.clear();

  Eigen::Matrix<double, 3, 1> P0, P1, V, P_buffer;
  int iterations;

  for (size_t i = 1; i < old_plan.size(); ++i)
  {
    P0 << old_plan[i - 1].pose.position.x, old_plan[i - 1].pose.position.y,
      old_plan[i - 1].pose.position.z;
    P1 << old_plan[i].pose.position.x, old_plan[i].pose.position.y, old_plan[i].pose.position.z;
    V = P1 - P0;

    iterations = int(V.norm() / distance);

    plan.push_back(old_plan[i - 1]);
    for (size_t j = 1; j < iterations; ++j)
    {
      P_buffer = P0 + j * (V / iterations);
      plan.push_back(eigenToPoseStamped(P_buffer));
    }
  }
}

void NavPiSamplingPlanner::addOrientation(std::vector<geometry_msgs::msg::PoseStamped>& path,
                                          geometry_msgs::msg::PoseStamped goal)
{
  Eigen::Matrix<double, 3, 1> a, b;
  a << 1, 0, 0;
  Eigen::Quaterniond q;

  for (size_t i = 0; i < path.size() - 1; ++i)
  {
    // Since most platforms just depend on yaw orientation we ignore all other
    // rotations by ignoreing the z coordinate
    b << (path[i + 1].pose.position.x - path[i].pose.position.x),
      (path[i + 1].pose.position.y - path[i].pose.position.y), 0.0;
    q = Eigen::Quaterniond().setFromTwoVectors(a, b);
    q.normalize();

    path[i].pose.orientation.x = q.x();
    path[i].pose.orientation.y = q.y();
    path[i].pose.orientation.z = q.z();
    path[i].pose.orientation.w = q.w();
  }
  path.back().pose.orientation = goal.pose.orientation;
}

void NavPiSamplingPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  auto params          = m_param_listener->get_params();
  double weight_data   = params.smoothing_data_weight;
  double weight_smooth = params.smoothing_weight;
  double tolerance     = params.smoothing_tolerance;
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

rclcpp_action::GoalResponse NavPiSamplingPlanner::handleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const navpi_interfaces::action::PlanPath::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "Received goal requested");
  if (m_state != State::ACTIVE)
  {
    RCLCPP_WARN(this->get_logger(), "Planner not active. Goal rejected.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
NavPiSamplingPlanner::handleCancel(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPiSamplingPlanner::handleAccepted(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
{
  std::thread{std::bind(&NavPiSamplingPlanner::executePlanPath, this, std::placeholders::_1),
              goal_handle}
    .detach();
}

void NavPiSamplingPlanner::executePlanPath(const std::shared_ptr<GoalHandlePlanPath> goal_handle)
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
  navpi_interfaces::msg::Result solver_result;

  using namespace std::placeholders;
  for (int i = 0; !plan_successful && i < retries; ++i)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Starting planning process. " << i + 1 << " of " << retries << " tries");
    if ((i + 1) > recovery_retries)
    {
      RCLCPP_INFO(this->get_logger(), "Starting recovery process.");
      geometry_msgs::msg::TransformStamped robot_pose;
      try
      {
        robot_pose = m_tf_buffer->lookupTransform(m_map_frame, m_robot_frame, tf2::TimePointZero);
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
      openvdb::Coord robot_index_coord;
      openvdb::Vec3d robot_world_coord;
      robot_world_coord.x() = robot_pose.transform.translation.x;
      robot_world_coord.y() = robot_pose.transform.translation.y;
      robot_world_coord.z() = robot_pose.transform.translation.z;

      auto params = m_param_listener->get_params();

      double resolution = 0.05;
      openvdb::CoordBBox m_clearing_box;
      m_clearing_box.min() = openvdb::Coord((int)(params.clearing_bounding_box.min.x / resolution),
                                            (int)(params.clearing_bounding_box.min.y / resolution),
                                            (int)(params.clearing_bounding_box.min.z / resolution));
      m_clearing_box.max() = openvdb::Coord((int)(params.clearing_bounding_box.max.x / resolution),
                                            (int)(params.clearing_bounding_box.max.y / resolution),
                                            (int)(params.clearing_bounding_box.max.z / resolution));

      std::unique_lock map_lock(*m_map_ptr->getMapMutex());
      robot_index_coord =
        openvdb::Coord::floor(m_map_ptr->getGrid()->worldToIndex(robot_world_coord));

      openvdb::FloatGrid::Accessor acc = m_map_ptr->getGrid()->getAccessor();

      for (int x = robot_index_coord.x() + m_clearing_box.min().x();
           x < robot_index_coord.x() + m_clearing_box.max().x();
           ++x)
      {
        for (int y = robot_index_coord.y() + m_clearing_box.min().y();
             y < robot_index_coord.y() + m_clearing_box.max().y();
             ++y)
        {
          for (int z = robot_index_coord.z() + m_clearing_box.min().z() + 1;
               z < robot_index_coord.z() + m_clearing_box.max().z();
               ++z)
          {
            acc.setActiveState(openvdb::Coord(x, y, z), false);
          }
          acc.setActiveState(openvdb::Coord(x, y, robot_index_coord.z() + m_clearing_box.min().z()),
                             true);
        }
      }
      map_lock.unlock();
    }

    makePlan(start_in_map, goal_in_map, 0.0, plan, cost, solver_result);
    if (solver_result.id == navpi_interfaces::msg::Result::GLOBAL_EXACT_SOLUTION ||
        solver_result.id == navpi_interfaces::msg::Result::GLOBAL_APPROX_SOLUTION)
    {
      plan_successful = true;
    }
  }

  if (plan_successful)
  {
    nav_msgs::msg::Path global_path;
    global_path.header.frame_id = m_map_frame;
    global_path.header.stamp    = this->now();
    global_path.poses           = plan;
    m_path_pub->publish(global_path);

    result->success        = true;
    result->planner_result = solver_result;
    result->path           = global_path;
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), solver_result.message.c_str());
    return;
  }
  else
  {
    result->success        = false;
    result->planner_result = solver_result;
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), solver_result.message.c_str());
  }
}

bool NavPiSamplingPlanner::isStateValid(const ob::State* state)
{
  m_used_nodes++;
  int x =
    state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value;
  int y =
    state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value;
  int z =
    state->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value;

  openvdb::Coord xyz(x, y, z);

  if (m_map_handle->isNodeValid(xyz))
  {
    if (m_debug_mode)
    {
      m_valid_sample_acc->setValueOn(xyz, 1.0);
    }
    return true;
  }
  if (m_debug_mode)
  {
    m_invalid_sample_acc->setValueOn(xyz, 1.0);
  }
  return false;
}

void NavPiSamplingPlanner::recoverGoalResponseCB(const GoalHandleRecover::SharedPtr& goal_handle)
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
void NavPiSamplingPlanner::recoverFeedbackCB(
  GoalHandleRecover::SharedPtr, const std::shared_ptr<const Recover::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "recovery feedback");
}
void NavPiSamplingPlanner::recoverResultCB(const GoalHandleRecover::WrappedResult& result) {}


void NavPiSamplingPlanner::publishDebugVisualization()
{
  std::cout << " Hey there i publish a lot of visual information about the planning process. Maybe "
               "I should move to the navpi itself so i can better incorporate the actual planning "
               "stuff .. but maybe not I think it is all here inside the map handle.."
            << std::endl;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
    m_map_handle->getCspaceGrid(), m_map_frame, cloud_msg);
  m_c_space_grid_pub->publish(cloud_msg);
  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
    m_map_handle->getValueGrid(), m_map_frame, cloud_msg);
  m_value_grid_pub->publish(cloud_msg);
  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
    m_valid_sample_grid, m_map_frame, cloud_msg);
  m_valid_pub->publish(cloud_msg);
  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
    m_invalid_sample_grid, m_map_frame, cloud_msg);
  m_invalid_pub->publish(cloud_msg);
}

void NavPiSamplingPlanner::publishStartAndGoal(openvdb::Coord start, openvdb::Coord goal)
{
  visualization_msgs::msg::Marker marker;
  openvdb::Vec3d start_world = m_map_handle->getValueGrid()->indexToWorld(start);

  marker.header.frame_id    = m_map_frame;
  marker.id                 = 0;
  marker.type               = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x            = 0.2;
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.2;
  marker.color.a            = 1.0;
  marker.color.g            = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.frame_locked       = true;
  marker.pose.position.x    = start_world.x();
  marker.pose.position.y    = start_world.y();
  marker.pose.position.z    = start_world.z();
  marker.action             = visualization_msgs::msg::Marker::ADD;
  m_start_marker_pub->publish(marker);

  openvdb::Vec3d goal_world = m_map_handle->getValueGrid()->indexToWorld(goal);
  marker.id                 = 1;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.pose.position.x    = goal_world.x();
  marker.pose.position.y    = goal_world.y();
  marker.pose.position.z    = goal_world.z();
  m_goal_marker_pub->publish(marker);
}

} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiSamplingPlanner)
