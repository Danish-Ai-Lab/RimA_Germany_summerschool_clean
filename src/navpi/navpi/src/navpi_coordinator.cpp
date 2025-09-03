#include "navpi/navpi_coordinator.hpp"

#include <iostream>
#include <memory>
#include <navpi_interfaces/action/detail/plan_and_execute_path__struct.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>

namespace navpi {
NavPiCoordinator::NavPiCoordinator(const rclcpp::NodeOptions& options)
  : Node("navpi_coordinator", options)
  , m_param_listener(
      std::make_shared<navpi_coordinator::ParamListener>(get_node_parameters_interface()))
{
  using namespace std::placeholders;


  m_plan_path_action_client    = rclcpp_action::create_client<PlanPath>(this, "plan_path");
  m_execute_path_action_client = rclcpp_action::create_client<ExecutePath>(this, "execute_path");
  m_plan_and_execute_cb_group  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  m_plan_and_execute_path_action_server = rclcpp_action::create_server<PlanAndExecutePath>(
    this,
    "plan_and_execute_path",
    std::bind(&NavPiCoordinator::handleGoal, this, _1, _2),
    std::bind(&NavPiCoordinator::handleCancel, this, _1),
    std::bind(&NavPiCoordinator::handleAccepted, this, _1),
    rcl_action_server_get_default_options(),
    m_plan_and_execute_cb_group);
}

rclcpp_action::GoalResponse NavPiCoordinator::handleGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const navpi_interfaces::action::PlanAndExecutePath::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal requested");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
NavPiCoordinator::handleCancel(const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  // TODO check if there is a case to reject the goal
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPiCoordinator::handleAccepted(
  const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle)
{
  using namespace std::placeholders;
  std::cout << "i accept it" << std::endl;
  std::thread{std::bind(&NavPiCoordinator::planAndExecutePlanPath, this, _1), goal_handle}.detach();
}

void NavPiCoordinator::planAndExecutePlanPath(
  const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle)
{
  using namespace std::placeholders;
  auto result = std::make_shared<PlanAndExecutePath::Result>();

  auto params = m_param_listener->get_params();

  const auto plan_and_execute_path_goal = goal_handle->get_goal();

  auto timeout = std::chrono::milliseconds((int)(params.global_planner_timeout * 1000));


  ExecutePath::Goal execute_path_goal = ExecutePath::Goal();

  auto plan_path_goal           = PlanPath::Goal();
  plan_path_goal.use_start_pose = plan_and_execute_path_goal->use_start_pose;
  plan_path_goal.start_pose     = plan_and_execute_path_goal->start_pose;
  plan_path_goal.target_pose    = plan_and_execute_path_goal->target_pose;


  RCLCPP_INFO(this->get_logger(), "Sending plan path goal");
  auto send_plan_path_goal_options = rclcpp_action::Client<PlanPath>::SendGoalOptions();
  send_plan_path_goal_options.goal_response_callback =
    std::bind(&NavPiCoordinator::planPathGoalResponseCB, this, _1);
  send_plan_path_goal_options.feedback_callback =
    std::bind(&NavPiCoordinator::planPathFeedbackCB, this, _1, _2);
  send_plan_path_goal_options.result_callback =
    std::bind(&NavPiCoordinator::planPathResultCB, this, _1);


  RCLCPP_INFO(this->get_logger(), "Sending goal to global planner action");
  auto plan_path_goal_handle =
    m_plan_path_action_client->async_send_goal(plan_path_goal, send_plan_path_goal_options);

  plan_path_goal_handle.wait();

  auto plan_path_result_future =
    m_plan_path_action_client->async_get_result(plan_path_goal_handle.get());
  if (plan_path_result_future.wait_for(timeout) != std::future_status::ready)
  {
    result->success           = false;
    result->planner_result.id = navpi_interfaces::msg::Result::GLOBAL_PLANNER_TIMEOUT;
    RCLCPP_ERROR(this->get_logger(), "Planner timed out during planning.");
    goal_handle->abort(result);
    return;
  }

  auto plan_path_result_handle = plan_path_result_future.get();
  if (plan_path_result_handle.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    result->success = false;
    RCLCPP_ERROR(this->get_logger(), "Planner did not find a path");
    result->success = navpi_interfaces::msg::Result::GLOBAL_NO_PATH_FOUND;
    goal_handle->abort(result);
    return;
  }

  auto plan_path_result = plan_path_result_handle.result;

  if (!plan_path_result->success)
  {
    result->success        = plan_path_result->success;
    result->planner_result = plan_path_result->planner_result;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Global planner action finished.");
  RCLCPP_INFO(this->get_logger(), "Setting up local planner");
  execute_path_goal.path = plan_path_result->path;

  if (!execute_path_goal.path.poses.empty())
  {
    auto send_execute_path_goal_options = rclcpp_action::Client<ExecutePath>::SendGoalOptions();
    send_execute_path_goal_options.goal_response_callback =
      std::bind(&NavPiCoordinator::executePathGoalResponseCB, this, _1);
    send_execute_path_goal_options.feedback_callback =
      std::bind(&NavPiCoordinator::executePathFeedbackCB, this, _1, _2);
    send_execute_path_goal_options.result_callback =
      std::bind(&NavPiCoordinator::executePathResultCB, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal to local planner action");
    auto execute_path_goal_handle = m_execute_path_action_client->async_send_goal(
      execute_path_goal, send_execute_path_goal_options);
    execute_path_goal_handle.wait();
    auto execute_path_result =
      m_execute_path_action_client->async_get_result(execute_path_goal_handle.get()).get().result;
    RCLCPP_INFO(this->get_logger(), "Local planner action finished.");

    if (!execute_path_result->success)
    {
      RCLCPP_INFO(this->get_logger(), "Execute path was not successful.");
      result->success        = execute_path_result->success;
      result->planner_result = execute_path_result->planner_result;
      goal_handle->abort(result);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Execute path was successful.");
      result->success          = execute_path_result->success;
      result->planner_result   = execute_path_result->planner_result;
      result->distance_to_goal = execute_path_result->distance_to_goal;
      result->angle_to_goal    = execute_path_result->angle_to_goal;
      result->final_pose       = execute_path_result->final_pose;

      goal_handle->succeed(result);
    }
  }
}

void NavPiCoordinator::planPathGoalResponseCB(const GoalHandlePlanPath::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal to plan path was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal to plan path was accepted by server, waiting for result");
  }
}
void NavPiCoordinator::planPathFeedbackCB(GoalHandlePlanPath::SharedPtr,
                                          const std::shared_ptr<const PlanPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "global planner feedback");
}
void NavPiCoordinator::planPathResultCB(const GoalHandlePlanPath::WrappedResult& result) {}

void NavPiCoordinator::executePathGoalResponseCB(
  const GoalHandleExecutePath::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal to execute plan was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),
                "Goal to execute plan was accepted by server, waiting for result");
  }
}
void NavPiCoordinator::executePathFeedbackCB(
  GoalHandleExecutePath::SharedPtr, const std::shared_ptr<const ExecutePath::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "local planner feedback");
}
void NavPiCoordinator::executePathResultCB(const GoalHandleExecutePath::WrappedResult& result) {}


} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiCoordinator)
