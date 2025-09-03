#ifndef NAVPI_RECOVERY_H
#define NAVPI_RECOVERY_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "vdb_mapping/OccupancyVDBMapping.hpp"

#include "navpi_interfaces/action/recover.hpp"


#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <navpi/navpi_recovery_parameters.hpp>
#include <eigen3/Eigen/Geometry>

namespace navpi {

class NavPiRecovery : public rclcpp_lifecycle::LifecycleNode
{
  using ValueType         = float;
  using ValueGridT        = openvdb::Grid<typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type>;
  using Recover           = navpi_interfaces::action::Recover;
  using GoalHandleRecover = rclcpp_action::ServerGoalHandle<Recover>;

  enum struct State
  {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FINALIZED
  };


public:
  explicit NavPiRecovery(const rclcpp::NodeOptions& options);
  virtual ~NavPiRecovery(){};

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  void setMap(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr);

  void recover(std::string& message);

  bool cancel();

  bool isInitialized() { return m_state == State::INACTIVE || m_state == State::ACTIVE; }

private:
  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const navpi_interfaces::action::Recover::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleRecover> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleRecover> goal_handle);

  void executeRecovery(const std::shared_ptr<GoalHandleRecover> goal_handle);

  rclcpp_action::Server<Recover>::SharedPtr m_plan_path_action_server;

  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;

  std::string m_map_frame;
  std::string m_robot_frame;

  State m_state = State::UNCONFIGURED;

  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  std::shared_ptr<navpi_recovery::ParamListener> m_param_listener;
};

} // namespace navpi
#endif /* NAVPI_RECOVERY_H */
