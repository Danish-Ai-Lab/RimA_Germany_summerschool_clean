#include "navpi/navpi_rmp_local_planner.hpp"
#include "navpi/navpi_diff_drive_controller_parameters.hpp"
#include "navpi/utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <navpi/navpi_rmp_local_planner_parameters.hpp>

#include <rumps/rmp.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <rumps/rmp_leaf.hpp>
#include <rumps/rmp_mapping.hpp>
#include <rumps/spaces.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <thread>

#include <pcl/impl/point_types.hpp>
#include <pcl/memory.h>

#include <rclcpp/event.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/server.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace navpi {

NavPiRMPLocalPlanner::NavPiRMPLocalPlanner(const rclcpp::NodeOptions &options)
    : LifecycleNode("navpi_rmp_local_planner", options), m_should_cancel(false),
      m_should_shutdown(false),
      m_param_listener_ptr(
          std::make_shared<navpi_rmp_local_planner::ParamListener>(
              get_node_parameters_interface())) {}

NavPiRMPLocalPlanner::CallbackReturn NavPiRMPLocalPlanner::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  m_params = m_param_listener_ptr->get_params();

  auto diff_drive_controller_param_listener =
      std::make_shared<navpi_diff_drive_controller::ParamListener>(
          get_node_parameters_interface(), "diff_drive_controller");
  m_diff_drive_controller = std::make_unique<navpi::NavPiDiffDriveController>(
      get_clock(), diff_drive_controller_param_listener, get_logger());

  auto path_sampler_param_listener =
      std::make_shared<navpi_path_sampler::ParamListener>(
          get_node_parameters_interface(), "path_sampler");

  m_path_sampler = std::make_unique<navpi::NavPiPathSampler>(
      this->get_clock(), path_sampler_param_listener, m_map_ptr,
      this->get_node_topics_interface());

  m_goal_publisher_ptr =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  m_cmd_vel_publisher =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  m_accel_publisher_ptr =
      this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);

  m_marker_publisher =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "force_field", 1);

  m_occ_grid_publisher =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_grid", 1);

  m_force_field_service = this->create_service<std_srvs::srv::Empty>(
      "publish_force_field",
      std::bind(&NavPiRMPLocalPlanner::publishForceFieldCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener_ptr =
      std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_tf_broadcaster_ptr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  m_execute_path_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_execute_path_action_server_ptr = rclcpp_action::create_server<ExecutePath>(
      this, "execute_path",
      [=](auto &&uuid, auto &&goal) {
        return handleExecutePathGoal(uuid, goal);
      },
      [=](auto &&goal_handle) { return handleExecutePathCancel(goal_handle); },
      [=](auto &&goal_handle) {
        return handleExecutePathAccepted(goal_handle);
      },
      rcl_action_server_get_default_options(), m_execute_path_cb_group);

  RCLCPP_DEBUG(this->get_logger(), "Created ExecutePath action server.");
  return CallbackReturn::SUCCESS;
}

NavPiRMPLocalPlanner::CallbackReturn NavPiRMPLocalPlanner::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  m_execute_path_thread_ptr =
      std::make_unique<std::thread>([=]() { executePathThread(); });
  return CallbackReturn::SUCCESS;
}
NavPiRMPLocalPlanner::CallbackReturn NavPiRMPLocalPlanner::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  m_should_shutdown = true;
  if (m_execute_path_thread_ptr->joinable()) {
    m_execute_path_thread_ptr->join();
  }
  return CallbackReturn::SUCCESS;
}
NavPiRMPLocalPlanner::CallbackReturn NavPiRMPLocalPlanner::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
  return CallbackReturn::SUCCESS;
}
NavPiRMPLocalPlanner::CallbackReturn NavPiRMPLocalPlanner::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
  return CallbackReturn::SUCCESS;
}

void NavPiRMPLocalPlanner::setMap(
    std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr) {
  m_map_ptr = std::move(map_ptr);
}

rclcpp_action::GoalResponse NavPiRMPLocalPlanner::handleExecutePathGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ExecutePath::Goal> &goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received an execute path request with the sec field of the "
              "header stamp beeing %d ",
              goal->path.header.stamp.sec);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse NavPiRMPLocalPlanner::handleExecutePathCancel(
    const std::shared_ptr<GoalHandleExecutePath> &goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel execute path");
  (void)goal_handle;
  {
    std::lock_guard<std::mutex> guard(m_next_goal_handle_mutex);
    if (m_next_goal_handle_ptr.has_value() &&
        m_next_goal_handle_ptr->get()->get_goal_id() ==
            goal_handle->get_goal_id()) {
      auto goal = m_next_goal_handle_ptr->get()->get_goal();
      auto goal_pose = goal->path.poses.back();

      auto result = std::make_shared<ExecutePath::Result>();
      geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();

      result->distance_to_goal =
          getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
      result->success = false;
      result->planner_result.id = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Goal was canceled before execution!";
      m_next_goal_handle_ptr->get()->canceled(result);
      m_next_goal_handle_ptr.reset();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  {
    std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
    if (m_current_goal_handle_ptr.has_value() &&
        m_current_goal_handle_ptr->get()->get_goal_id() ==
            goal_handle->get_goal_id()) {
      m_should_cancel = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void NavPiRMPLocalPlanner::handleExecutePathAccepted(
    const std::shared_ptr<GoalHandleExecutePath> &goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Local planner received new goal, creating handle");
  std::lock_guard<std::mutex> guard(m_next_goal_handle_mutex);

  if (m_next_goal_handle_ptr.has_value()) {
    auto goal = m_next_goal_handle_ptr->get()->get_goal();
    auto goal_pose = goal->path.poses.back();

    auto result = std::make_shared<ExecutePath::Result>();
    geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();

    result->distance_to_goal =
        getEuclidDistanceBetweenPoses(goal_pose, current_pose);
    result->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
    result->success = false;
    result->planner_result.id = navpi_interfaces::msg::Result::LOCAL_CANCELED;
    result->planner_result.message = "Goal was preempted before execution!";
    m_next_goal_handle_ptr->get()->abort(result);
    m_next_goal_handle_ptr.reset();
  }
  goal_handle->execute();
  m_next_goal_handle_ptr = std::make_optional(goal_handle);
}

void NavPiRMPLocalPlanner::executePathThread() {
  RCLCPP_INFO(this->get_logger(), "Starting background thread!");
  std::shared_ptr<GoalHandleExecutePath> goal_to_execute;
  while (!m_should_shutdown) {
    {
      // std::lock_guard<std::mutex> guard(this->m_next_goal_handle_mutex);
      std::unique_lock<std::mutex> guard(m_next_goal_handle_mutex);

      if (m_next_goal_handle_ptr.has_value()) {
        goal_to_execute = m_next_goal_handle_ptr.value();
        {
          std::lock_guard<std::mutex> guard2(m_current_goal_handle_mutex);
          m_current_goal_handle_ptr.swap(m_next_goal_handle_ptr);
        }
        m_next_goal_handle_ptr.reset();
        guard.unlock();
        executePath(goal_to_execute);
      } else {
        guard.unlock();
        auto clock = this->get_clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock, 5000,
                              "No next goal available, waiting!");
        // TODO: We still do active waiting here, the conditon_variables for
        // event based are broken with RCLCPP.
        rclcpp::sleep_for(1s);
        continue;
      }
    }
  }
}

void NavPiRMPLocalPlanner::executePath(
    const std::shared_ptr<GoalHandleExecutePath> &goal_handle) {
  RCLCPP_DEBUG(this->get_logger(), "Executing goal");

  m_params = m_param_listener_ptr->get_params();

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ExecutePath::Feedback>();
  auto result = std::make_shared<ExecutePath::Result>();

  RCLCPP_DEBUG(this->get_logger(),
               "Execute: Created goal handle and action messages");

  rclcpp::Rate loop_rate(m_params.loop_rate);

  if (goal->path.poses.size() == 0) {
    result->success = false;
    result->planner_result.id =
        navpi_interfaces::msg::Result::LOCAL_INVALID_PATH;
    result->planner_result.message = "Provided path has length 0.";
    goal_handle->abort(result);
    return;
  }

  m_path = goal->path;
  m_path_sampler->setPath(m_path);

  RCLCPP_DEBUG(this->get_logger(), "Execute: Filled member variables");
  unsigned long path_pose_number = goal->path.poses.size();

  RCLCPP_DEBUG(this->get_logger(), "Execute: Number of poses in path: %lu",
               path_pose_number);

  RCLCPP_DEBUG(this->get_logger(), "Execute: Saved path in right type");
  geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();
  const geometry_msgs::msg::PoseStamped goal_pose = goal->path.poses.back();
  RCLCPP_DEBUG(this->get_logger(), "Execute: Got current and goal pose");
  RCLCPP_DEBUG(this->get_logger(), "Goal Pose: x: %f y: %f z: %f",
               goal_pose.pose.position.x, goal_pose.pose.position.y,
               goal_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(), "Current Pose: x: %f y: %f z: %f",
               current_pose.pose.position.x, current_pose.pose.position.y,
               current_pose.pose.position.z);

  bool initial_alignment_done = !m_params.initial_alignment;

  m_diff_drive_controller->resetPID();
  rclcpp::Time last_call = now();
  rumps::SE3::Vec q = rumps::SE3::Vec::Zero();
  rumps::SE3::Vec q_dot = rumps::SE3::Vec::Zero();
  rumps::SE3::Vec q_dot_robot = rumps::SE3::Vec::Zero();
  while (goalIsNotReached(goal_pose, current_pose)) {
    m_params = m_param_listener_ptr->get_params();
    if (m_next_goal_handle_ptr.has_value()) {
      result->success = false;
      result->planner_result.id = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Preempted by new movement goal!";
      result->distance_to_goal =
          getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
      result->final_pose = current_pose;
      goal_handle->abort(result);

      std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
      m_current_goal_handle_ptr.reset();
      return;
    }
    if (m_should_cancel) {
      result->success = false;
      result->planner_result.id = navpi_interfaces::msg::Result::LOCAL_CANCELED;
      result->planner_result.message = "Canceled on user request!";
      result->distance_to_goal =
          getEuclidDistanceBetweenPoses(goal_pose, current_pose);
      result->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
      result->final_pose = current_pose;
      goal_handle->canceled(result);

      std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
      m_current_goal_handle_ptr.reset();
      m_should_cancel = false;
      return;
    }

    rclcpp::Time next_call = now();
    rclcpp::Duration period = (next_call - last_call);
    last_call = next_call;
    double period_ns = period.nanoseconds();

    RCLCPP_INFO(get_logger(), "Control iteration took %f seconds",
                period.seconds());
    double tolerance = 0.05 * m_params.loop_rate;
    if (period.seconds() > 1.0 / m_params.loop_rate + tolerance) {
      RCLCPP_ERROR(get_logger(),
                   "Planner is taking too long, target period is %f",
                   1.0 / m_params.loop_rate);
    }

    current_pose = tfLookupCurrentPose();
    m_diff_drive_controller->setPose(current_pose.pose);

    RCLCPP_DEBUG(this->get_logger(), "Execute: Iteration in while loop");
    geometry_msgs::msg::PoseStamped next_pose =
        m_path_sampler->sampleNextPoseInPath(current_pose);

    updatePreTargetFrame(next_pose);

    computeConfiguration(current_pose, q);

    Eigen::Isometry3d next_pose_t;
    Eigen::Isometry3d goal_pose_t;
    Eigen::fromMsg(next_pose.pose, next_pose_t);
    Eigen::fromMsg(goal_pose.pose, goal_pose_t);
    Eigen::Vector3d next_pose_vec = next_pose_t.translation();
    Eigen::Matrix3d next_pose_orientation = next_pose_t.rotation();
    Eigen::Matrix3d goal_orientation = goal_pose_t.rotation();
    RCLCPP_INFO_STREAM(get_logger(), "Next Pose: " << next_pose_vec.transpose());

    double distance_to_goal =
        getEuclidDistanceBetweenPoses(goal_pose, current_pose);

    if (distance_to_goal > m_params.goal_distance_tolerance) {
      if (!initial_alignment_done) {
        double angle_diff = getAngleBetweenPoses(next_pose, current_pose);
        if (angle_diff > m_params.goal_angular_distance) {
          RCLCPP_INFO(get_logger(), "Performing initial alignment, current error: %f", angle_diff);
          handleOrientationAlignment(q, q_dot, q_dot_robot,
                                     next_pose_orientation, period_ns);
        } else {
          RCLCPP_INFO(get_logger(), "Initial alignment done");
          initial_alignment_done = true;
        }
      } else {
        // follow path
        RCLCPP_INFO(get_logger(), "Following path");
        handlePathFollowing(q, q_dot, q_dot_robot, next_pose_vec, period_ns);
      }
    } else {
      // final alignment
      handleOrientationAlignment(q, q_dot, q_dot_robot, goal_orientation,
                                 period_ns);
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "Computed cmd_vel: " << q_dot_robot.transpose());

    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.frame_id = m_params.robot_frame;
    cmd_vel_msg.header.stamp = get_clock()->now();
    cmd_vel_msg.twist.linear.x = q_dot_robot(0);
    cmd_vel_msg.twist.linear.y = q_dot_robot(1);
    cmd_vel_msg.twist.linear.z = q_dot_robot(2);
    cmd_vel_msg.twist.angular.x = q_dot_robot(3);
    cmd_vel_msg.twist.angular.y = q_dot_robot(4);
    cmd_vel_msg.twist.angular.z = q_dot_robot(5);

    m_cmd_vel_publisher->publish(cmd_vel_msg);

    feedback = populateFeedback(current_pose, goal_pose);
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(get_logger(), "------------");
    loop_rate.sleep();
  }

  result->success = true;
  result->planner_result.id = navpi_interfaces::msg::Result::SUCCESS;
  result->planner_result.message = "Successfully executed the planned route.";
  result->distance_to_goal =
      getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  result->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
  result->final_pose = current_pose;

  goal_handle->succeed(result);

  std::lock_guard<std::mutex> guard(m_current_goal_handle_mutex);
  m_current_goal_handle_ptr.reset();
}

bool NavPiRMPLocalPlanner::goalIsNotReached(
    const geometry_msgs::msg::PoseStamped &goal_pose,
    const geometry_msgs::msg::PoseStamped &current_pose) {
  double distance_euclid =
      getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  double angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
  RCLCPP_DEBUG(this->get_logger(), "Goal Pose: x: %f y: %f z: %f",
               goal_pose.pose.position.x, goal_pose.pose.position.y,
               goal_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(), "Current Pose: x: %f y: %f z: %f",
               current_pose.pose.position.x, current_pose.pose.position.y,
               current_pose.pose.position.z);
  RCLCPP_DEBUG(this->get_logger(), "Euclid distance to goal: %f",
               distance_euclid);
  RCLCPP_DEBUG(this->get_logger(), "Angle to goal: %f", angle_to_goal);
  if (distance_euclid < m_params.goal_distance_tolerance &&
      abs(angle_to_goal) < m_params.goal_angular_distance) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Execute:  Euclid distance smaller than 0.1, goal reached");
    return false;
  }
  RCLCPP_DEBUG(this->get_logger(),
               "Execute: Euclid distance bigger than 0.1, continuing planner");
  return true;
}

geometry_msgs::msg::PoseStamped NavPiRMPLocalPlanner::tfLookupCurrentPose() {
  RCLCPP_DEBUG(this->get_logger(), "Start tf lookup for current pose");
  try {
    auto body_pose = geometry_msgs::msg::PoseStamped();
    body_pose.header.frame_id = m_params.robot_frame;
    geometry_msgs::msg::PoseStamped pose;
    m_tf_buffer->transform(body_pose, pose, m_params.map_frame);
    return pose;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Error in Transformation from %s to %s: Error message %s",
                m_params.robot_frame.c_str(), m_params.map_frame.c_str(),
                ex.what());
    return geometry_msgs::msg::PoseStamped();
  }
}

std::shared_ptr<navpi_interfaces::action::ExecutePath::Feedback>
NavPiRMPLocalPlanner::populateFeedback(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose) {
  auto feedback = std::make_shared<ExecutePath::Feedback>();
  feedback->distance_to_goal =
      getEuclidDistanceBetweenPoses(goal_pose, current_pose);
  feedback->angle_to_goal = getAngleBetweenPoses(goal_pose, current_pose);
  feedback->current_pose = current_pose;
  return feedback;
}

tf2::Stamped<tf2::Transform> NavPiRMPLocalPlanner::poseToTransform(
    const geometry_msgs::msg::PoseStamped &pose_in) {
  auto transform_out = tf2::Stamped<tf2::Transform>();
  // transform_out.stamp_ = pose_in.header.stamp;
  transform_out.frame_id_ = pose_in.header.frame_id;
  tf2::Transform temp;
  temp.setOrigin(tf2::Vector3(pose_in.pose.position.x, pose_in.pose.position.y,
                              pose_in.pose.position.z));
  temp.setRotation(
      tf2::Quaternion(pose_in.pose.orientation.x, pose_in.pose.orientation.y,
                      pose_in.pose.orientation.z, pose_in.pose.orientation.w));
  transform_out.setData(temp);
  return transform_out;
}

void NavPiRMPLocalPlanner::updatePreTargetFrame(
    const geometry_msgs::msg::PoseStamped &pre_target) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = m_params.map_frame;
  t.child_frame_id = "pre_target";
  t.transform.translation.x = pre_target.pose.position.x;
  t.transform.translation.y = pre_target.pose.position.y;
  t.transform.translation.z = pre_target.pose.position.z;
  t.transform.rotation.x = pre_target.pose.orientation.x;
  t.transform.rotation.y = pre_target.pose.orientation.y;
  t.transform.rotation.z = pre_target.pose.orientation.z;
  t.transform.rotation.w = pre_target.pose.orientation.w;
  m_tf_broadcaster_ptr->sendTransform(t);
}

void NavPiRMPLocalPlanner::computeConfiguration(
    const geometry_msgs::msg::PoseStamped &current_pose, rumps::SE3::Vec &q) {
  // extract rpy
  tf2::Quaternion current_quat;
  tf2::convert(current_pose.pose.orientation, current_quat);
  tf2::Matrix3x3 current_mat(current_quat);
  Eigen::Vector3d current_rpy;
  current_mat.getRPY(current_rpy[0], current_rpy[1], current_rpy[2]);

  q.setZero();
  q << current_pose.pose.position.x, current_pose.pose.position.y,
      current_pose.pose.position.z, current_rpy[0], current_rpy[1],
      current_rpy[2];
}

void NavPiRMPLocalPlanner::publishAccelMsg(const rumps::SE3::Vec &q_ddot) {
  geometry_msgs::msg::Accel accel;
  accel.linear.x = q_ddot[0];
  accel.linear.y = q_ddot[1];
  accel.linear.z = q_ddot[2];
  accel.angular.x = q_ddot[3];
  accel.angular.y = q_ddot[4];
  accel.angular.z = q_ddot[5];

  geometry_msgs::msg::AccelStamped msg;
  msg.header.frame_id = m_params.robot_frame;
  msg.header.stamp = get_clock()->now();

  try {
    geometry_msgs::msg::TransformStamped t = m_tf_buffer->lookupTransform(
        m_params.robot_frame, m_params.map_frame, rclcpp::Time(0));
    tf2::doTransform(accel.linear, msg.accel.linear, t);
    tf2::doTransform(accel.angular, msg.accel.angular, t);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Error in Transformation from %s to %s: Error message %s",
                m_params.map_frame.c_str(), m_params.robot_frame.c_str(),
                ex.what());
    return;
  }

  m_accel_publisher_ptr->publish(msg);
}

void NavPiRMPLocalPlanner::publishForceField(rumps::Root<rumps::SE3> &root,
                                             const rumps::SE3::Vec &q,
                                             const rumps::SE3::Vec &q_dot) {
  rumps::SE3::Vec min, max;
  min = q - m_params.debug.force_field.size * rumps::SE3::Vec::Ones();
  max = q + m_params.debug.force_field.size * rumps::SE3::Vec::Ones();

  publishForceField(root, min, max, m_params.debug.force_field.resolution, 5.0,
                    q_dot);
}

void NavPiRMPLocalPlanner::publishForceField(rumps::Root<rumps::SE3> &root,
                                             const rumps::SE3::Vec &min,
                                             const rumps::SE3::Vec &max,
                                             double resolution, double timeout,
                                             const rumps::SE3::Vec &q_dot) {
  using namespace visualization_msgs::msg;

  MarkerArray msg;
  size_t num_markers =
      (max[0] - min[0]) / resolution * (max[1] - min[1]) / resolution;
  msg.markers.reserve(num_markers);

  Marker marker;
  marker.header.frame_id = m_params.map_frame;
  marker.header.stamp = get_clock()->now();
  marker.ns = "navpi_rmp_local_planner_force_field";
  marker.type = Marker::ARROW;

  // Set arrow thickness
  marker.scale.x = 0.02; // shaft diameter
  marker.scale.y = 0.05; // head diameter
  marker.scale.z = 0.1;  // head length

  // Color
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(timeout);

  size_t index = 0;

  for (double xi = min[0]; xi < max[0]; xi += resolution) {
    for (double yi = min[1]; yi < max[1]; yi += resolution) {
      rumps::SE3::Vec q_hat;
      q_hat[0] = xi;
      q_hat[1] = yi;

      rumps::SE3::Vec a = root.solve(q_hat, q_dot);
      if (a.norm() > 1.0) {
        marker.color.g = 1.0;
      } else {
        marker.color.g = 0.0;
      }

      a *= m_params.debug.force_field.arrow_scaler;

      marker.id = index;
      // convert to marker msg
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = q_hat.x();
      p_start.y = q_hat.y();
      p_start.z = q_hat.z();

      p_end.x = q_hat.x() + a.x();
      p_end.y = q_hat.y() + a.y();
      p_end.z = q_hat.z() + a.z();

      marker.points.clear();
      marker.points.push_back(p_start);
      marker.points.push_back(p_end);

      msg.markers.emplace_back(marker);
      index++;
    }
  }

  m_marker_publisher->publish(msg);
}
void NavPiRMPLocalPlanner::collectObstacles(
    std::vector<Eigen::Vector3d> &obstacles, const Eigen::Vector3d &pose) {
  Eigen::Vector3d min_boundaries, max_boundaries;
  min_boundaries = pose - Eigen::Vector3d::Ones() * m_params.sampling_distance;
  max_boundaries = pose + Eigen::Vector3d::Ones() * m_params.sampling_distance;

  vdb_mapping::OccupancyVDBMapping::GridT::Ptr local_grid =
      m_map_ptr->getMapSectionGrid(min_boundaries, max_boundaries,
                                   Eigen::Matrix<double, 4, 4>::Identity());

  nav_msgs::msg::OccupancyGrid occ_grid;
  // TODO resolution is hardcoded
  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
      local_grid, m_params.map_frame, occ_grid, -0.3, 0.5, 0.05);

  Eigen::Vector3d map_origin =
      pose - Eigen::Vector3d(occ_grid.info.width, occ_grid.info.height, 0.0) *
                 occ_grid.info.resolution / 2.0;

  occ_grid.header.frame_id = m_params.map_frame;
  occ_grid.header.stamp = get_clock()->now();
  occ_grid.info.origin.position = Eigen::toMsg(map_origin);
  m_occ_grid_publisher->publish(occ_grid);

  visualization_msgs::msg::MarkerArray marker_msg;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = m_params.map_frame;
  marker.header.stamp = get_clock()->now();
  marker.ns = "rmp_local_planner_obstacles";
  marker.id = 0;
  marker.scale.x = 0.07;
  marker.scale.y = 0.07;
  marker.scale.z = 0.07;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.b = 1.0;
  marker.type = visualization_msgs::msg::Marker::POINTS;

  // collect obstalces
  for (size_t i = 0; i < occ_grid.info.width; ++i) {
    for (size_t j = 0; j < occ_grid.info.height; ++j) {
      double val = occ_grid.data[i + j * occ_grid.info.width];
      // count both occupied and unknown cells as obstacles
      if (val > m_params.obstacle_certainty_thresh || val < 0) {
        Eigen::Vector3d x;
        x << i * occ_grid.info.resolution, j * occ_grid.info.resolution, 0.0;
        x += map_origin;
        obstacles.emplace_back(x);
        marker.points.emplace_back(Eigen::toMsg(x));
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Number of obstacles found: %zu", obstacles.size());

  marker_msg.markers.push_back(marker);
  m_marker_publisher->publish(marker_msg);
}

void NavPiRMPLocalPlanner::publishForceFieldCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  m_params = m_param_listener_ptr->get_params();
  geometry_msgs::msg::PoseStamped current_pose = tfLookupCurrentPose();

  Eigen::Vector3d direction;
  rumps::SE3::Vec q, q_dot;
  rumps::Position2D::Vec cmd_vel = rumps::Diff::Vec::Zero();
  computeConfiguration(current_pose, q);

  // add the goal as attractor
  Eigen::Vector3d goal(m_params.debug.force_field.dummy_goal_position.x,
                       m_params.debug.force_field.dummy_goal_position.y, 0.0);

  std::vector<Eigen::Vector3d> obstacles;
  if (m_params.position_repulsor.enabled) {
    collectObstacles(obstacles, q.head<3>());
  }

  rumps::Root<rumps::SE3> root;
  buildTree(obstacles, goal, root);

  rumps::SE3::Vec min, max;
  max << m_params.debug.force_field.size, m_params.debug.force_field.size, 0;
  min = -max;

  publishForceField(root, min, max, m_params.debug.force_field.resolution);
}

void NavPiRMPLocalPlanner::buildTree(
    const std::vector<Eigen::Vector3d> &obstacles, const Eigen::Vector3d &goal,
    rumps::Root<rumps::SE3> &root) {

  root.clearChildren();

  // create mapping to position space
  auto &position_mapping = root.addChild<rumps::Position2DMap>();

  auto &obstacle_parent_node =
      position_mapping.addChild<rumps::Node<rumps::Position2D>>();

  auto &damper = position_mapping.addChild<rumps::Damper<rumps::Position2D>>(
      m_params.damper.lambda);

  // add the goal as attractor
  auto &goal_attractor =
      position_mapping.addChild<rumps::PositionAttractor<rumps::Position2D>>(
          goal.head<2>(), m_params.position_attractor.eta,
          m_params.position_attractor.alpha,
          m_params.position_attractor.psi_thresh);

  auto &regularizer = root.addChild<rumps::TikhonovRegularizer<rumps::SE3>>(
      m_params.regularizer.lambda);

  for (const auto &obstacle : obstacles) {
    obstacle_parent_node.addChild<rumps::PositionRepulsor<rumps::Position2D>>(
        obstacle.head<2>(), m_params.position_repulsor.epsilon,
        m_params.position_repulsor.eta, m_params.position_repulsor.alpha,
        m_params.position_repulsor.p);
  }
}

void NavPiRMPLocalPlanner::handlePathFollowing(const rumps::SE3::Vec &q,
                                               rumps::SE3::Vec &q_dot,
                                               rumps::SE3::Vec &q_dot_robot,
                                               Eigen::Vector3d &goal,
                                               double period_ns) {
  std::vector<Eigen::Vector3d> obstacles;
  if (m_params.position_repulsor.enabled) {
    collectObstacles(obstacles, q.head<3>());
  }
  RCLCPP_INFO(get_logger(), "Number of obstacles found: %zu", obstacles.size());

  RCLCPP_INFO_STREAM(get_logger(), "Q: " << q.transpose());
  RCLCPP_INFO_STREAM(get_logger(), "Q_DOT: " << q_dot.transpose());

  // build rmp tree
  // create new root node
  rumps::Root<rumps::SE3> root;
  buildTree(obstacles, goal, root);

  rumps::SE3::Vec q_ddot = root.solve(q, q_dot);

  publishAccelMsg(q_ddot);
  RCLCPP_INFO_STREAM(get_logger(), "Q_DDOT: " << q_ddot.transpose());

  m_diff_drive_controller->updateCommand(q_ddot, q_dot, q_dot_robot, period_ns);

  if (m_params.debug.force_field.enabled) {
    publishForceField(root, q, q_dot);
  }
}

void NavPiRMPLocalPlanner::handleOrientationAlignment(
    const rumps::SE3::Vec &q, rumps::SE3::Vec &q_dot,
    rumps::SE3::Vec &q_dot_robot, Eigen::Matrix3d &target_orientation,
    double period_ns) {
  Eigen::Vector3d orientation_vec = target_orientation.eulerAngles(0, 1, 2);
  m_diff_drive_controller->alignOrientation(q_dot_robot, orientation_vec,
                                            period_ns);
}

} // namespace navpi

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navpi::NavPiRMPLocalPlanner)
