#ifndef NAVPI_SAFETY_H
#define NAVPI_SAFETY_H

#include "navpi/navpi_safety_field.hpp"
#include <navpi/navpi_local_planner_parameters.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace navpi {

class NavPiSafety
{
public:
  NavPiSafety(rclcpp::Clock::SharedPtr clock,
              std::shared_ptr<navpi_local_planner::ParamListener> param_listener,
              std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
              rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if);
  NavPiSafety()                              = delete;
  NavPiSafety(NavPiSafety&&)                 = default;
  NavPiSafety(const NavPiSafety&)            = default;
  NavPiSafety& operator=(NavPiSafety&&)      = default;
  NavPiSafety& operator=(const NavPiSafety&) = default;
  ~NavPiSafety()                             = default;

  void clampSafety(geometry_msgs::msg::TwistStamped& cmd_vel,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles);

  void createSafetyFields();
  void createSafetyFieldsMarkerMsg();
  void checkFields(pcl::PointXYZ& p);

private:
  std::map<std::string, SafetyField> m_safety_fields;
  std::string m_robot_frame;

  visualization_msgs::msg::MarkerArray m_safety_field_vis_msg;

  rclcpp::Clock::SharedPtr m_clock;

  std::shared_ptr<navpi_local_planner::ParamListener> m_param_listener;
  navpi_local_planner::Params m_params;

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr m_topic_if;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_safety_fields_vis_pub;
};

} // namespace navpi
#endif // !NAVPI_SAFETY_H
