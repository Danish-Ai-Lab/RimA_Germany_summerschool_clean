#include "navpi//navpi_safety.hpp"

namespace navpi {


NavPiSafety::NavPiSafety(rclcpp::Clock::SharedPtr clock,
                         std::shared_ptr<navpi_local_planner::ParamListener> param_listener,
                         std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
                         rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if)
  : m_clock(clock)
  , m_param_listener(param_listener)
  , m_topic_if(topic_if)
{
  m_params      = m_param_listener->get_params();
  m_robot_frame = m_params.robot_frame;

  m_safety_fields_vis_pub = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
    m_topic_if, "safety_fields_debug", 1);


  createSafetyFields();
  createSafetyFieldsMarkerMsg();
}

void NavPiSafety::clampSafety(geometry_msgs::msg::TwistStamped& cmd_vel,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles)
{
  for (auto& p : obstacles->points)
  {
    checkFields(p);
  }

  if (m_safety_fields["front"].isSafetyTriggered() && cmd_vel.twist.linear.x > 0)
  {
    cmd_vel.twist.linear.x = 0;
  }
  if (m_safety_fields["rear"].isSafetyTriggered() && cmd_vel.twist.linear.x < 0)
  {
    cmd_vel.twist.linear.x = 0;
  }
  if (m_safety_fields["left"].isSafetyTriggered() && cmd_vel.twist.linear.y > 0)
  {
    cmd_vel.twist.linear.y = 0;
  }
  if (m_safety_fields["right"].isSafetyTriggered() && cmd_vel.twist.linear.y < 0)
  {
    cmd_vel.twist.linear.y = 0;
  }
  if (m_safety_fields["front_left"].isSafetyTriggered() && cmd_vel.twist.angular.z > 0)
  {
    cmd_vel.twist.angular.z = 0;
  }
  if (m_safety_fields["rear_right"].isSafetyTriggered() && cmd_vel.twist.angular.z > 0)
  {
    cmd_vel.twist.angular.z = 0;
  }
  if (m_safety_fields["front_right"].isSafetyTriggered() && cmd_vel.twist.angular.z < 0)
  {
    cmd_vel.twist.angular.z = 0;
  }
  if (m_safety_fields["rear_left"].isSafetyTriggered() && cmd_vel.twist.angular.z < 0)
  {
    cmd_vel.twist.angular.z = 0;
  }

  // update marker msgs
  int i = 0;
  for (auto const& field : m_safety_fields)
  {
    visualization_msgs::msg::Marker& marker = m_safety_field_vis_msg.markers.at(i);
    marker.header.stamp                     = m_clock->now();
    marker.action                           = visualization_msgs::msg::Marker::MODIFY;
    if (field.second.isSafetyTriggered())
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
    }
    i++;
  }

  m_safety_fields_vis_pub->publish(m_safety_field_vis_msg);
}

void NavPiSafety::createSafetyFields()
{
  m_params = m_param_listener->get_params();
  // Create safety fields
  Eigen::Matrix<double, 3, 1> min_p, max_p;
  // Creating front and rear field
  double offset;
  double length;
  double width;
  offset = m_params.safety.translation.x.offset;
  length = m_params.safety.translation.x.length;
  width  = m_params.safety.translation.x.width;

  min_p << offset, -1 * width, -10;
  max_p << offset + length, width, 10;
  m_safety_fields["front"] = SafetyField(min_p, max_p);

  min_p << -1 * offset - length, -1 * width, -10;
  max_p << -1 * offset, width, 10;
  m_safety_fields["rear"] = SafetyField(min_p, max_p);

  // Creating left and right field
  offset = m_params.safety.translation.y.offset;
  length = m_params.safety.translation.y.length;
  width  = m_params.safety.translation.y.width;

  min_p << -1 * width, offset, -10;
  max_p << width, offset + length, 10;
  m_safety_fields["left"] = SafetyField(min_p, max_p);

  min_p << -1 * width, -1 * offset - length, -10;
  max_p << width, -1 * offset, 10;
  m_safety_fields["right"] = SafetyField(min_p, max_p);

  // create rotation field
  double offset_x;
  double offset_y;
  double size_x;
  double size_y;
  offset_x = m_params.safety.rotation.offset_x;
  offset_y = m_params.safety.rotation.offset_y;
  length   = m_params.safety.rotation.length;
  width    = m_params.safety.rotation.width;

  min_p << offset_x, offset_y, -10;
  max_p << offset_x + size_x, offset_y + size_y, 10;
  m_safety_fields["front_left"] = SafetyField(min_p, max_p);

  min_p << offset_x, -1 * offset_y - size_y, -10;
  max_p << offset_x + size_x, -1 * offset_y, 10;
  m_safety_fields["front_right"] = SafetyField(min_p, max_p);

  min_p << -1 * offset_x - size_x, -1 * offset_y, -10;
  max_p << -1 * offset_x, offset_y + size_y, 10;
  m_safety_fields["rear_left"] = SafetyField(min_p, max_p);

  min_p << -1 * offset_x - size_x, -1 * offset_y - size_y, -10;
  max_p << -1 * offset_x, -1 * offset_y, 10;
  m_safety_fields["rear_right"] = SafetyField(min_p, max_p);
}
void NavPiSafety::createSafetyFieldsMarkerMsg()
{
  int i = 0;

  for (const auto& field : m_safety_fields)
  {
    visualization_msgs::msg::Marker marker;
    Eigen::Matrix<double, 3, 1> scale = field.second.getMaxP() - field.second.getMinP();
    Eigen::Matrix<double, 3, 1> pose  = field.second.getMinP() + scale / 2;


    marker.header.frame_id    = m_robot_frame;
    marker.header.stamp       = m_clock->now();
    marker.ns                 = "bounding_box";
    marker.id                 = i++;
    marker.type               = visualization_msgs::msg::Marker::CUBE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = pose.x();
    marker.pose.position.y    = pose.y();
    marker.pose.position.z    = pose.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = scale.x();
    marker.scale.y            = scale.y();
    marker.scale.z            = 0.5; // og is like -10 to 10
    marker.color.a            = 0.5;
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    m_safety_field_vis_msg.markers.push_back(marker);
  }

  m_safety_fields_vis_pub->publish(m_safety_field_vis_msg);
}

void NavPiSafety::checkFields(pcl::PointXYZ& p)
{
  for (auto& field : m_safety_fields)
  {
    if (field.second.containsPoint(p))
    {
      field.second.triggerSafety();
    }
    else
    {
      field.second.releaseSafety();
    }
  }
}
} // namespace navpi
