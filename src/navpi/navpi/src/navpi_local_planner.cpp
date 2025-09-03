#include "navpi/navpi_local_planner.hpp"
#include "navpi/utils.hpp"
#include <navpi/navpi_local_planner_parameters.hpp>

#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <pcl/impl/point_types.hpp>
#include <pcl/memory.h>

#include <rclcpp/event.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/server.hpp>

using namespace std::chrono_literals;

namespace navpi {


NavPiLocalPlanner::NavPiLocalPlanner(const rclcpp::NodeOptions& options)
  : LifecycleNode("navpi_local_planner", options)
  , m_should_cancel(false)
  , m_should_shutdown(false)
  , m_param_listener_ptr(
      std::make_shared<navpi_local_planner::ParamListener>(get_node_parameters_interface()))
{
}

NavPiLocalPlanner::CallbackReturn
NavPiLocalPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  auto path_sampler_param_listener =
      std::make_shared<navpi_path_sampler::ParamListener>(
          get_node_parameters_interface());

  m_path_sampler = std::make_unique<navpi::NavPiPathSampler>(
    this->get_clock(), path_sampler_param_listener, m_map_ptr, this->get_node_topics_interface());
  m_obstacle_avoidance = std::make_unique<navpi::NavPiObstacleAvoidance>(
    this->get_clock(), m_param_listener_ptr, m_map_ptr, this->get_node_topics_interface());
  m_base_controller =
    std::make_unique<navpi::NavPiBaseController>(this->get_clock(), m_param_listener_ptr);
  m_safety = std::make_unique<navpi::NavPiSafety>(
    this->get_clock(), m_param_listener_ptr, m_map_ptr, this->get_node_topics_interface());


  auto params          = m_param_listener_ptr->get_params();
  m_angle_tolerance    = params.goal_angular_distance;
  m_distance_tolerance = params.goal_distance_tolerance;
  m_loop_rate          = params.loop_rate;
  m_robot_frame        = params.robot_frame;
  m_map_frame          = params.map_frame;
  m_sampler_only_mode  = params.sampler_only_mode;


  m_goal_publisher_ptr = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  m_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  m_tf_buffer          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener_ptr    = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_tf_broadcaster_ptr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  m_execute_path_cb_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_execute_path_action_server_ptr = rclcpp_action::create_server<ExecutePath>(
    this,
    "execute_path",
    [=](auto&& uuid, auto&& goal) { return handleExecutePathGoal(uuid, goal); },
    [=](auto&& goal_handle) { return handleExecutePathCancel(goal_handle); },
    [=](auto&& goal_handle) { return handleExecutePathAccepted(goal_handle); },
    rcl_action_server_get_default_options(),
    m_execute_path_cb_group);

  RCLCPP_DEBUG(this->get_logger(), "Created ExecutePath action server.");
  return CallbackReturn::SUCCESS;
}

NavPiLocalPlanner::CallbackReturn
NavPiLocalPlanner::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  m_execute_path_thread_ptr = std::make_unique<std::thread>([=]() { executePathThread(); });
  return CallbackReturn::SUCCESS;
}
NavPiLocalPlanner::CallbackReturn
NavPiLocalPlanner::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_should_shutdown = true;
  if (m_execute_path_thread_ptr->joinable())
  {
    m_execute_path_thread_ptr->join();
  }
  return CallbackReturn::SUCCESS;
}
NavPiLocalPlanner::CallbackReturn
NavPiLocalPlanner::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}
NavPiLocalPlanner::CallbackReturn
NavPiLocalPlanner::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

void NavPiLocalPlanner::setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr)
{
  m_map_ptr = std::move(map_ptr);
}

rclcpp_action::GoalResponse
NavPiLocalPlanner::handleExecutePathGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const ExecutePath::Goal>& goal)
{
  RCLCPP_INFO(this->get_logger(),
              "Received an execute path request with the sec field of the header stamp beeing %d ",
              goal->path.header.stamp.sec);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse NavPiLocalPlanner::handleExecutePathCancel(
  const std::shared_ptr<GoalHandleExecutePath>& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel execute path");
  (void)goal_handle;
  {
    std::lock_guard<std::mutex> guard(m_next_goal_handle_mutex);
    if (m_next_goal_handle_ptr.has_value() &&
        m_next_goal_handle_ptr->get()->get_goal_id() == goal_handle->get_goal_id())
    {
      auto goal      = m_next_goal_handle_ptr->get()->get_goal();
      auto goal_pose = goal->path.poses.back();

      auto result                                  = std::make_shared<ExecutePath::Result>();
      geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();

      result->distance_to_goal       = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal          = getAngleBetweenPoses(goal_pose, current_pose);
      result->success                = false;
      result->planner_result.id      = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Goal was canceled before execution!";
      m_next_goal_handle_ptr->get()->canceled(result);
      m_next_goal_handle_ptr.reset();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  {
    std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
    if (m_current_goal_handle_ptr.has_value() &&
        m_current_goal_handle_ptr->get()->get_goal_id() == goal_handle->get_goal_id())
    {
      m_should_cancel = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void NavPiLocalPlanner::handleExecutePathAccepted(
  const std::shared_ptr<GoalHandleExecutePath>& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Local planner received new goal, creating handle");
  std::lock_guard<std::mutex> guard(m_next_goal_handle_mutex);

  if (m_next_goal_handle_ptr.has_value())
  {
    auto goal      = m_next_goal_handle_ptr->get()->get_goal();
    auto goal_pose = goal->path.poses.back();

    auto result                                  = std::make_shared<ExecutePath::Result>();
    geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();

    result->distance_to_goal       = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
    result->angle_to_goal          = getAngleBetweenPoses(goal_pose, current_pose);
    result->success                = false;
    result->planner_result.id      = navpi_interfaces::msg::Result::LOCAL_CANCELED;
    result->planner_result.message = "Goal was preempted before execution!";
    m_next_goal_handle_ptr->get()->abort(result);
    m_next_goal_handle_ptr.reset();
  }
  goal_handle->execute();
  m_next_goal_handle_ptr = std::make_optional(goal_handle);
}


void NavPiLocalPlanner::executePathThread()
{
  RCLCPP_INFO(this->get_logger(), "Starting background thread!");
  std::shared_ptr<GoalHandleExecutePath> goal_to_execute;
  while (!m_should_shutdown)
  {
    {
      // std::lock_guard<std::mutex> guard(this->m_next_goal_handle_mutex);
      std::unique_lock<std::mutex> guard(m_next_goal_handle_mutex);

      if (m_next_goal_handle_ptr.has_value())
      {
        goal_to_execute = m_next_goal_handle_ptr.value();
        {
          std::lock_guard<std::mutex> guard2(m_current_goal_handle_mutex);
          m_current_goal_handle_ptr.swap(m_next_goal_handle_ptr);
        }
        m_next_goal_handle_ptr.reset();
        guard.unlock();
        executePath(goal_to_execute);
      }
      else
      {
        guard.unlock();
        auto clock = this->get_clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock, 5000, "No next goal available, waiting!");
        // TODO: We still do active waiting here, the conditon_variables for event based are broken
        // with RCLCPP.
        rclcpp::sleep_for(1s);
        continue;
      }
    }
  }
}

void NavPiLocalPlanner::executePath(const std::shared_ptr<GoalHandleExecutePath>& goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ExecutePath::Feedback>();
  auto result   = std::make_shared<ExecutePath::Result>();

  RCLCPP_DEBUG(this->get_logger(), "Execute: Created goal handle and action messages");

  rclcpp::Rate loop_rate(m_loop_rate);

  if (goal->path.poses.size() == 0)
  {
    result->success                = false;
    result->planner_result.id      = navpi_interfaces::msg::Result::LOCAL_INVALID_PATH;
    result->planner_result.message = "Provided path has length 0.";
    goal_handle->abort(result);
    return;
  }

  m_path = goal->path;
  m_path_sampler->setPath(m_path);

  RCLCPP_DEBUG(this->get_logger(), "Execute: Filled member variables");
  unsigned long path_pose_number = goal->path.poses.size();

  RCLCPP_DEBUG(this->get_logger(), "Execute: Number of poses in path: %lu", path_pose_number);

  RCLCPP_DEBUG(this->get_logger(), "Execute: Saved path in right type");
  geometry_msgs::msg::PoseStamped current_pose    = tfLookupCurrentPose();
  const geometry_msgs::msg::PoseStamped goal_pose = goal->path.poses.back();
  RCLCPP_DEBUG(this->get_logger(), "Execute: Got current and goal pose");
  RCLCPP_DEBUG(this->get_logger(),
               "Goal Pose: x: %f y: %f z: %f",
               goal_pose.pose.position.x,
               goal_pose.pose.position.y,
               goal_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(),
               "Current Pose: x: %f y: %f z: %f",
               current_pose.pose.position.x,
               current_pose.pose.position.y,
               current_pose.pose.position.z);
  geometry_msgs::msg::PoseStamped next_pose;

  m_base_controller->updatePidParams();

  while (goalIsNotReached(goal_pose, current_pose))
  {
    if (m_next_goal_handle_ptr.has_value())
    {
      result->success                = false;
      result->planner_result.id      = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Preempted by new movement goal!";
      result->distance_to_goal       = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal          = getAngleBetweenPoses(goal_pose, current_pose);
      result->final_pose             = current_pose;
      goal_handle->abort(result);

      std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
      m_current_goal_handle_ptr.reset();
      return;
    }
    if (m_should_cancel)
    {
      result->success                = false;
      result->planner_result.id      = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Canceled on user request!";
      result->distance_to_goal       = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal          = getAngleBetweenPoses(goal_pose, current_pose);
      result->final_pose             = current_pose;
      goal_handle->canceled(result);


      std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
      m_current_goal_handle_ptr.reset();
      m_should_cancel = false;
      return;
    }
    current_pose = tfLookupCurrentPose();
    RCLCPP_DEBUG(this->get_logger(), "Execute: Iteration in while loop");
    next_pose = m_path_sampler->sampleNextPoseInPath(current_pose);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = this->now();
    t.header.frame_id         = m_map_frame;
    t.child_frame_id          = "pre_target";
    t.transform.translation.x = next_pose.pose.position.x;
    t.transform.translation.y = next_pose.pose.position.y;
    t.transform.translation.z = next_pose.pose.position.z;
    t.transform.rotation.x    = next_pose.pose.orientation.x;
    t.transform.rotation.y    = next_pose.pose.orientation.y;
    t.transform.rotation.z    = next_pose.pose.orientation.z;
    t.transform.rotation.w    = next_pose.pose.orientation.w;
    m_tf_broadcaster_ptr->sendTransform(t);

    if (m_sampler_only_mode)
    {
      m_goal_publisher_ptr->publish(next_pose);
    }
    else
    {
      // Obstacle Avoidance
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles;
      obstacles = m_obstacle_avoidance->adjustTarget(next_pose, m_path.poses.back());

      // Controller
      geometry_msgs::msg::TwistStamped cmd_vel_output;
      cmd_vel_output = m_base_controller->updateCommand(next_pose);

      // Safety
      m_safety->clampSafety(cmd_vel_output, obstacles);


      m_cmd_vel_publisher->publish(cmd_vel_output);
    }

    feedback = populateFeedback(current_pose, goal_pose);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  result->success                = true;
  result->planner_result.id      = navpi_interfaces::msg::Result::SUCCESS;
  result->planner_result.message = "Successfully executed the planned route.";
  result->distance_to_goal       = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  result->angle_to_goal          = getAngleBetweenPoses(goal_pose, current_pose);
  result->final_pose             = current_pose;

  goal_handle->succeed(result);

  std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
  m_current_goal_handle_ptr.reset();
}

bool NavPiLocalPlanner::goalIsNotReached(const geometry_msgs::msg::PoseStamped& goal_pose,
                                         const geometry_msgs::msg::PoseStamped& current_pose)
{
  double distance_euclid = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  double angle_to_goal   = getAngleBetweenPoses(goal_pose, current_pose);
  RCLCPP_DEBUG(this->get_logger(),
               "Goal Pose: x: %f y: %f z: %f",
               goal_pose.pose.position.x,
               goal_pose.pose.position.y,
               goal_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(),
               "Current Pose: x: %f y: %f z: %f",
               current_pose.pose.position.x,
               current_pose.pose.position.y,
               current_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(), "Euclid distance to goal: %f", distance_euclid);
  RCLCPP_DEBUG(this->get_logger(), "Angle to goal: %f", angle_to_goal);
  if (distance_euclid < m_distance_tolerance && abs(angle_to_goal) < m_angle_tolerance)
  {
    RCLCPP_DEBUG(this->get_logger(), "Execute:  Euclid distance smaller than 0.1, goal reached");
    return false;
  }
  RCLCPP_DEBUG(this->get_logger(), "Execute: Euclid distance bigger than 0.1, continuing planner");
  return true;
}

geometry_msgs::msg::PoseStamped NavPiLocalPlanner::tfLookupCurrentPose()
{
  RCLCPP_DEBUG(this->get_logger(), "Start tf lookup for current pose");
  try
  {
    auto body_pose            = geometry_msgs::msg::PoseStamped();
    body_pose.header.frame_id = m_robot_frame;
    geometry_msgs::msg::PoseStamped pose;
    m_tf_buffer->transform(body_pose, pose, m_map_frame);
    return pose;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(),
                "Error in Transformation from %s to %s: Error message %s",
                m_robot_frame.c_str(),
                m_map_frame.c_str(),
                ex.what());
    return geometry_msgs::msg::PoseStamped();
  }
}

std::shared_ptr<navpi_interfaces::action::ExecutePath::Feedback>
NavPiLocalPlanner::populateFeedback(const geometry_msgs::msg::PoseStamped& current_pose,
                                    const geometry_msgs::msg::PoseStamped& goal_pose)
{
  auto feedback              = std::make_shared<ExecutePath::Feedback>();
  feedback->distance_to_goal = getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  feedback->angle_to_goal    = getAngleBetweenPoses(goal_pose, current_pose);
  feedback->current_pose     = current_pose;
  return feedback;
}

tf2::Stamped<tf2::Transform>
NavPiLocalPlanner::poseToTransform(const geometry_msgs::msg::PoseStamped& pose_in)
{
  auto transform_out = tf2::Stamped<tf2::Transform>();
  // transform_out.stamp_ = pose_in.header.stamp;
  transform_out.frame_id_ = pose_in.header.frame_id;
  tf2::Transform temp;
  temp.setOrigin(
    tf2::Vector3(pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z));
  temp.setRotation(tf2::Quaternion(pose_in.pose.orientation.x,
                                   pose_in.pose.orientation.y,
                                   pose_in.pose.orientation.z,
                                   pose_in.pose.orientation.w));
  transform_out.setData(temp);
  return transform_out;
}

} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiLocalPlanner)
