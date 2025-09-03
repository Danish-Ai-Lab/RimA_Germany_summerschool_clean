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
#include <tf2_ros/transform_listener.h>


namespace navpi {
NavPiSamplingPlanner::NavPiSamplingPlanner(const rclcpp::NodeOptions& options)
  : LifecycleNode("navpi_sampling_planner", options)
  , m_param_listener(
      std::make_shared<navpi_global_planner::ParamListener>(get_node_parameters_interface()))
{
}


NavPiSamplingPlanner::CallbackReturn
NavPiSamplingPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto params   = m_param_listener->get_params();
  m_map_frame   = params.map_frame;
  m_robot_frame = params.robot_frame;

  if (m_map_ptr == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Failure to configure sampling Planner: Map not set.");
    return CallbackReturn::FAILURE;
  }


  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_path_pub    = this->create_publisher<nav_msgs::msg::Path>("path", 1);

  using namespace std::placeholders;
  m_plan_path_action_server = rclcpp_action::create_server<PlanPath>(
    this,
    "plan_path",
    std::bind(
      &NavPiSamplingPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavPiSamplingPlanner::handleCancel, this, std::placeholders::_1),
    std::bind(&NavPiSamplingPlanner::handleAccepted, this, std::placeholders::_1));

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
                                    std::string& message)
{
  auto setup_t1 = std::chrono::high_resolution_clock::now();


  auto params  = m_param_listener->get_params();
  m_vdb_grid   = m_map_ptr->getGrid();
  m_map_handle = std::make_shared<MapHandle>(m_vdb_grid);
  auto space(std::make_shared<IntegerStateSpace>());
  ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
  si->setStateValidityChecker(
    [this](const ob::State* state) { return NavPiSamplingPlanner::isStateValid(state); });

  m_ompl_setup = std::make_shared<og::SimpleSetup>(si);

  name = "prm";

  if (name == "rrt")
  {
    m_ompl_setup->setPlanner(std::make_shared<og::RRT>(m_ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_connect")
  {
    m_ompl_setup->setPlanner(std::make_shared<og::RRTConnect>(m_ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_star")
  {
    m_ompl_setup->setPlanner(std::make_shared<og::RRTstar>(m_ompl_setup->getSpaceInformation()));
  }
  else if (name == "rrt_star_connect")
  {
    m_ompl_setup->setPlanner(
      std::make_shared<og::RRTstarConnect>(m_ompl_setup->getSpaceInformation()));
  }
  else if (name == "prm")
  {
    m_ompl_setup->setPlanner(std::make_shared<og::PRM>(m_ompl_setup->getSpaceInformation()));
  }
  else
  {
    m_ompl_setup->setPlanner(std::make_shared<og::RRTConnect>(m_ompl_setup->getSpaceInformation()));
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
  m_map_handle->generateValidNode(start_index);
  std::cout << "Start Index adapted: " << start_index << std::endl;

  ob::ScopedState<> ompl_start(m_ompl_setup->getStateSpace());
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value =
    start_index.x();
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value =
    start_index.y();
  ompl_start->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value =
    start_index.z();


  openvdb::Coord goal_index =
    openvdb::math::Coord::round(m_map_handle->getValueGrid()->worldToIndex(
      openvdb::Vec3d(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)));
  std::cout << "Goal Index initial: " << goal_index << std::endl;
  m_map_handle->generateValidNode(goal_index);
  std::cout << "Goal Index adapted: " << goal_index << std::endl;

  m_goals.push_back(goal_index);


  m_planning_times.clear();
  for (auto& goal : m_goals)
  {
    if (m_ompl_setup->getPlanner())
    {
      m_ompl_setup->getPlanner()->clearQuery();
    }

    auto solve_t1 = std::chrono::high_resolution_clock::now();

    // ob::ScopedState<> ompl_goal(m_ompl_setup->getStateSpace());
    // ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value
    // = goal_index.x();
    // ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value
    // = goal_index.y();
    // ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value
    // = goal_index.z();
    ob::ScopedState<> ompl_goal(m_ompl_setup->getStateSpace());
    ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(0)->value =
      goal.x();
    ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(1)->value =
      goal.y();
    ompl_goal->as<IntegerStateSpace::StateType>()->as<ob::DiscreteStateSpace::StateType>(2)->value =
      goal.z();

    m_ompl_setup->setStartAndGoalStates(ompl_start, ompl_goal);

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

    for (int i = 0; i < runs; i++)
    {
      if (m_cancel_requested)
      {
        m_cancel_requested = false;
        return;
      }
      if (m_ompl_setup->getPlanner())
      {
        m_ompl_setup->getPlanner()->clearQuery();
      }
      m_ompl_setup->solve(time);
    }

    auto solve_t2                                      = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> solve_ms = solve_t2 - solve_t1;
    m_planning_times.push_back(solve_ms.count());
  }

  std::size_t ns = m_ompl_setup->getProblemDefinition()->getSolutionCount();
  OMPL_INFORM("FOUND %d solutions", (int)ns);

  if (m_ompl_setup->haveSolutionPath())
  {
    og::PathGeometric& p = m_ompl_setup->getSolutionPath();
    m_ompl_setup->getPathSimplifier()->simplifyMax(p);

    geometry_msgs::msg::PoseStamped pose_buffer;
    pose_buffer.header.frame_id = "map";
    // pose_buffer.header.stamp       = ros::Time::now();
    pose_buffer.pose.orientation.w = 1;
    std::cout << "State count: " << p.getStateCount() << std::endl;
    for (size_t i = 0; i < p.getStateCount(); i++)
    {
      std::cout << "FUUU" << std::endl;
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

      std::cout << index_pose << std::endl;
      openvdb::Vec3d world_pose   = m_map_handle->getValueGrid()->indexToWorld(index_pose);
      pose_buffer.pose.position.x = world_pose.x();
      pose_buffer.pose.position.y = world_pose.y();
      pose_buffer.pose.position.z = world_pose.z();
      plan.push_back(pose_buffer);
      std::cout << "im the end" << std::endl;

      nav_msgs::msg::Path global_path;
      global_path.header.frame_id = "map";
      // global_path.header.stamp    = ros::Time::now();
      global_path.poses = plan;
      m_path_pub->publish(global_path);
    }
  }
  else
  {
    std::cout << " NO solution path found" << std::endl;
  }
  int bla = 0;
  for (auto& t : m_planning_times)
  {
    std::cout << "Planner " << ++bla << ": " << std::endl;
    std::cout << t << " ms" << std::endl;
  }

  for (auto& bla : m_goals)
  {
    std::cout << bla << std::endl;
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
  openvdb::Vec3d world_coord = m_vdb_grid->indexToWorld(coord);
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

  return openvdb::Coord::floor(m_vdb_grid->worldToIndex(world_coord));
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

void NavPiSamplingPlanner::addOrientation(std::vector<geometry_msgs::msg::PoseStamped>& path)
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

void NavPiSamplingPlanner::smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path)
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
  auto goal   = goal_handle->get_goal();
  auto result = std::make_shared<PlanPath::Result>();

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
    // TODO error handling
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> plan;
  double cost;
  std::string message;
  makePlan(transformToPose(robot_pose), goal->target_pose, 0.0, plan, cost, message);

  if (plan.size() == 0)
  {
    result->success                = false;
    result->planner_result.id      = navpi_interfaces::msg::Result::GLOBAL_NO_PATH_FOUND;
    result->planner_result.message = message;
    goal_handle->succeed(result);

    RCLCPP_WARN(this->get_logger(), "Planner found empty path!");
    return;
  }

  nav_msgs::msg::Path global_path;
  global_path.header = robot_pose.header;
  global_path.poses  = plan;
  m_path_pub->publish(global_path);

  result->success                = true;
  result->planner_result.id      = navpi_interfaces::msg::Result::SUCCESS;
  result->planner_result.message = message;
  result->path                   = global_path;
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Planner execution done.");
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

  // TODO not sure why it is here anymore but its super aweful
  if (x == 42 && y == 42 && z == 42)
    return false;

  openvdb::Coord xyz(x, y, z);
  // std::cout << "Checking Voxel: " << xyz<< std::endl;

  // if (hasGround(xyz, m_value_grid_acc))
  // if (isFree(xyz, m_cspace_grid_acc) && hasGround(xyz, m_value_grid_acc))
  // if (isFree(xyz, m_value_grid_acc) && hasGround(xyz, m_value_grid_acc))
  // if(m_map_handle->isNodeValid(xyz,m_map_handle->getCspaceGrid()))
  //  if(isFree(xyz, m_cspace_grid_acc))
  // if(m_map_handle->isFree(xyz, m_map_handle->getCspaceAcc() ))
  if (m_map_handle->generateValidNode(xyz, 2))
  {
    // m_valid_sampled_acc->setValueOn(xyz, 1.0);
    return true;
  }
  // m_invalid_sampled_acc->setValueOn(xyz, 1.0);
  return false;
}


} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiSamplingPlanner)
