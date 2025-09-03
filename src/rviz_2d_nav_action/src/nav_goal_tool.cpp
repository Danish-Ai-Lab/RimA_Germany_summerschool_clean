// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  "Felix Exner" <exner@fzi.de>
 * \date    2022-07-15
 *
 */
//----------------------------------------------------------------------

#include <iomanip>

#include "rviz_2d_nav_action_navpi/nav_goal_tool.hpp"

#include <QMessageBox>
#include <navpi_interfaces/action/detail/plan_and_execute_path__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/create_client.hpp>


namespace rviz_2d_nav_action_navpi {

NavGoalTool::NavGoalTool()
{
  shortcut_key_ = 'm';

  m_action_namespace_ptr.reset(
    new rviz_common::properties::StringProperty("ActionNamespace",
                                                "navpi/plan_and_execute_path",
                                                "The namespace in which the action lives",
                                                getPropertyContainer(),
                                                SLOT(createActionClient()),
                                                this));
  m_name_ptr.reset(new rviz_common::properties::StringProperty(
    "Name", "2D Nav Goal", "The display name", getPropertyContainer(), SLOT(updateName()), this));
}

NavGoalTool::~NavGoalTool() {}

void NavGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.0F, 0.45F, 0.28F, 1.0F);
  setName(m_name_ptr->getString());
  createActionClient();
  QObject::connect(this, &NavGoalTool::gotResult, this, &NavGoalTool::displayError);
}

void NavGoalTool::onPoseSet(double x, double y, double theta)
{
  using namespace std::placeholders;

  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  auto goal = navpi_interfaces::action::PlanAndExecutePath::Goal();

  goal.target_pose.pose.orientation = tf2::toMsg(quat);
  goal.target_pose.pose.position.x  = x;
  goal.target_pose.pose.position.y  = y;
  goal.target_pose.header.frame_id  = fixed_frame;
  goal.target_pose.header.stamp     = context_->getClock()->now();

  RCLCPP_INFO(
    rclcpp::get_logger("rviz_2d_nav_action_navpi"),
    "Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
    "Angle: %.3f\n",
    fixed_frame.c_str(),
    goal.target_pose.pose.position.x,
    goal.target_pose.pose.position.y,
    goal.target_pose.pose.position.z,
    goal.target_pose.pose.orientation.x,
    goal.target_pose.pose.orientation.y,
    goal.target_pose.pose.orientation.z,
    goal.target_pose.pose.orientation.w,
    theta);


  auto send_goal_options =
    rclcpp_action::Client<navpi_interfaces::action::PlanAndExecutePath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&NavGoalTool::activeCb, this, _1);
  send_goal_options.feedback_callback      = std::bind(&NavGoalTool::feedbackCb, this, _1, _2);
  send_goal_options.result_callback        = std::bind(&NavGoalTool::doneCb, this, _1);
  this->m_ac_ptr->async_send_goal(goal, send_goal_options);
}


void NavGoalTool::doneCb(const GoalHandleNavpi::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("rviz_2d_nav_action_navpi"), "Goal was aborted");
      emit gotResult();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rclcpp::get_logger("rviz_2d_nav_action_navpi"), "Goal was canceled");
      emit gotResult();
      return;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("rviz_2d_nav_action_navpi"), "Unknown result code");
      emit gotResult();
      return;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_2d_nav_action_navpi"),
                     "Final distance: " << std::setprecision(5) << result.result->distance_to_goal);
}

void NavGoalTool ::displayError()
{
  QMessageBox::critical(nullptr,
                        QString("Executing goal failed!"),
                        QString("Navigating to the requested goal failed. Either no plan was found "
                                "or executing the plan failed."));
}

void NavGoalTool::activeCb(GoalHandleNavpi::SharedPtr future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_2d_nav_action_navpi"), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rviz_2d_nav_action_navpi"),
                "Goal accepted by server, waiting for result");
  }
}

void NavGoalTool::feedbackCb(
  GoalHandleNavpi::SharedPtr,
  const navpi_interfaces::action::PlanAndExecutePath::Feedback::ConstSharedPtr feedback_ptr)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_2d_nav_action_navpi"),
                     "Distance to goal: " << std::setprecision(2)
                                          << feedback_ptr->distance_to_goal);
}

void NavGoalTool::createActionClient()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_ac_ptr  = rclcpp_action::create_client<navpi_interfaces::action::PlanAndExecutePath>(
    node, m_action_namespace_ptr->getStdString());
}

void NavGoalTool::updateName()
{
  setName(m_name_ptr->getString());
}
} // namespace rviz_2d_nav_action_navpi

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_nav_action_navpi::NavGoalTool, rviz_common::Tool)
