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

#ifndef RVIZ_2D_NAV_ACTION_NAV_GOAL_TOOL_H_INCLUDED
#define RVIZ_2D_NAV_ACTION_NAV_GOAL_TOOL_H_INCLUDED

#include <QObject>

#include <navpi_interfaces/action/detail/plan_and_execute_path__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navpi_interfaces/action/plan_and_execute_path.hpp>
#include <rclcpp_action/client.hpp>

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
namespace rviz_2d_nav_action_navpi {
class NavGoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  using GoalHandleNavpi =
    rclcpp_action::ClientGoalHandle<navpi_interfaces::action::PlanAndExecutePath>;
  NavGoalTool();
  ~NavGoalTool() override;

  void onInitialize() override;


protected:
  void onPoseSet(double x, double y, double theta) override;


private Q_SLOTS:
  void createActionClient();
  void displayError();
  void updateName();

Q_SIGNALS:
  void gotResult();

private:
  void doneCb(const GoalHandleNavpi::WrappedResult& result);
  void activeCb(GoalHandleNavpi::SharedPtr future);
  void feedbackCb(
    GoalHandleNavpi::SharedPtr,
    const navpi_interfaces::action::PlanAndExecutePath::Feedback::ConstSharedPtr feedback_ptr);

  rclcpp_action::Client<navpi_interfaces::action::PlanAndExecutePath>::SharedPtr m_ac_ptr;
  std::unique_ptr<rviz_common::properties::StringProperty> m_action_namespace_ptr;
  std::unique_ptr<rviz_common::properties::StringProperty> m_name_ptr;
};
} // namespace rviz_2d_nav_action
#endif // ifndef RVIZ_2D_NAV_ACTION_NAV_GOAL_TOOL_H_INCLUDED
