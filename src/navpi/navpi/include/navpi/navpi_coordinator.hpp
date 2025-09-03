#ifndef NAVPI_COORDINATOR_H
#define NAVPI_COORDINATOR_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"

#include "navpi_interfaces/action/execute_path.hpp"
#include "navpi_interfaces/action/plan_and_execute_path.hpp"
#include "navpi_interfaces/action/plan_path.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <navpi/navpi_coordinator_parameters.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>


namespace navpi {

class NavPiCoordinator : public rclcpp::Node
{
  using PlanAndExecutePath           = navpi_interfaces::action::PlanAndExecutePath;
  using GoalHandlePlanAndExecutePath = rclcpp_action::ServerGoalHandle<PlanAndExecutePath>;
  using PlanPath                     = navpi_interfaces::action::PlanPath;
  using GoalHandlePlanPath           = rclcpp_action::ClientGoalHandle<PlanPath>;
  using ExecutePath                  = navpi_interfaces::action::ExecutePath;
  using GoalHandleExecutePath        = rclcpp_action::ClientGoalHandle<ExecutePath>;

public:
  explicit NavPiCoordinator(const rclcpp::NodeOptions& options);
  virtual ~NavPiCoordinator(){};


private:
  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const navpi_interfaces::action::PlanAndExecutePath::Goal> goal);

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle);

  void planAndExecutePlanPath(const std::shared_ptr<GoalHandlePlanAndExecutePath> goal_handle);

  void planPathGoalResponseCB(const GoalHandlePlanPath::SharedPtr& goal_handle);
  void planPathFeedbackCB(GoalHandlePlanPath::SharedPtr,
                          const std::shared_ptr<const PlanPath::Feedback> feedback);
  void planPathResultCB(const GoalHandlePlanPath::WrappedResult& result);

  void executePathGoalResponseCB(const GoalHandleExecutePath::SharedPtr& goal_handle);
  void executePathFeedbackCB(GoalHandleExecutePath::SharedPtr,
                             const std::shared_ptr<const ExecutePath::Feedback> feedback);
  void executePathResultCB(const GoalHandleExecutePath::WrappedResult& result);

  rclcpp_action::Server<PlanAndExecutePath>::SharedPtr m_plan_and_execute_path_action_server;
  rclcpp_action::Client<PlanPath>::SharedPtr m_plan_path_action_client;
  rclcpp_action::Client<ExecutePath>::SharedPtr m_execute_path_action_client;


  std::shared_ptr<navpi_coordinator::ParamListener> m_param_listener;

  rclcpp::CallbackGroup::SharedPtr m_plan_and_execute_cb_group;
};

} // namespace navpi
#endif /* NAVPI_COORDINATOR */
