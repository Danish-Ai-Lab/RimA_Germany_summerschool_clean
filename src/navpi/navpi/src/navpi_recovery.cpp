#include "navpi/navpi_recovery.hpp"

#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace navpi {
NavPiRecovery::NavPiRecovery(const rclcpp::NodeOptions& options)
  : LifecycleNode("navpi_recovery", options)
  , m_param_listener(
      std::make_shared<navpi_recovery::ParamListener>(get_node_parameters_interface()))
{
}


NavPiRecovery::CallbackReturn
NavPiRecovery::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto params   = m_param_listener->get_params();
  m_map_frame   = params.map_frame;
  m_robot_frame = params.robot_frame;

  if (m_map_ptr == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Failure to configure recovery: Map not set.");
    return CallbackReturn::FAILURE;
  }

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  using namespace std::placeholders;
  m_plan_path_action_server = rclcpp_action::create_server<Recover>(
    this,
    "recover",
    std::bind(&NavPiRecovery::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavPiRecovery::handleCancel, this, std::placeholders::_1),
    std::bind(&NavPiRecovery::handleAccepted, this, std::placeholders::_1));

  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiRecovery::CallbackReturn
NavPiRecovery::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::ACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiRecovery::CallbackReturn
NavPiRecovery::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::INACTIVE;
  return CallbackReturn::SUCCESS;
}
NavPiRecovery::CallbackReturn
NavPiRecovery::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::UNCONFIGURED;
  return CallbackReturn::SUCCESS;
}
NavPiRecovery::CallbackReturn
NavPiRecovery::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  m_state = State::FINALIZED;
  return CallbackReturn::SUCCESS;
}

void NavPiRecovery::setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr)
{
  m_map_ptr = map_ptr;
}

rclcpp_action::GoalResponse
NavPiRecovery::handleGoal(const rclcpp_action::GoalUUID& uuid,
                          std::shared_ptr<const navpi_interfaces::action::Recover::Goal> goal)
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
NavPiRecovery::handleCancel(const std::shared_ptr<GoalHandleRecover> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavPiRecovery::handleAccepted(const std::shared_ptr<GoalHandleRecover> goal_handle)
{
  std::thread{std::bind(&NavPiRecovery::executeRecovery, this, std::placeholders::_1), goal_handle}
    .detach();
}

void NavPiRecovery::executeRecovery(const std::shared_ptr<GoalHandleRecover> goal_handle)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  auto result = std::make_shared<Recover::Result>();


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
  robot_index_coord = openvdb::Coord::floor(m_map_ptr->getGrid()->worldToIndex(robot_world_coord));

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

  result->success = true;
  goal_handle->succeed(result);
}


} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiRecovery)
