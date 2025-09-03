#ifndef NAVPI_OBSTACLE_AVOIDANCE_H
#define NAVPI_OBSTACLE_AVOIDANCE_H

#include <navpi/navpi_local_planner_parameters.hpp>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include "vdb_mapping_ros2/VDBMappingTools.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Eigenvalues>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace navpi {

class NavPiObstacleAvoidance
{
public:
  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;


  NavPiObstacleAvoidance(rclcpp::Clock::SharedPtr clock,
                         std::shared_ptr<navpi_local_planner::ParamListener> param_listener,
                         std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
                         rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if);
  NavPiObstacleAvoidance()                                         = delete;
  NavPiObstacleAvoidance(NavPiObstacleAvoidance&&)                 = default;
  NavPiObstacleAvoidance(const NavPiObstacleAvoidance&)            = default;
  NavPiObstacleAvoidance& operator=(NavPiObstacleAvoidance&&)      = default;
  NavPiObstacleAvoidance& operator=(const NavPiObstacleAvoidance&) = default;
  ~NavPiObstacleAvoidance()                                        = default;

  pcl::PointCloud<pcl::PointXYZ>::Ptr adjustTarget(geometry_msgs::msg::PoseStamped& next_pose,
                                                   geometry_msgs::msg::PoseStamped& final_pose);

private:
  std::pair<Eigen::Vector3d, Eigen::Vector3d>
  line_fitting(const std::vector<Eigen::Vector3d>& points);


  sensor_msgs::msg::LaserScan
  extractNavigationCorridor(geometry_msgs::msg::TransformStamped& base_to_map_tf,
                            geometry_msgs::msg::TransformStamped& map_to_base_tf,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles);

  float calculateDistanceFromSegment(const std::vector<Eigen::Vector3d>& segment_points);

  void calculateRewards(sensor_msgs::msg::LaserScan& scan,
                        geometry_msgs::msg::PoseStamped& target,
                        geometry_msgs::msg::TransformStamped& base_to_map_tf,
                        geometry_msgs::msg::TransformStamped& map_to_base_tf);

  double getNewTargetAngle(const sensor_msgs::msg::LaserScan& scan);

  rclcpp::Clock::SharedPtr m_clock;
  std::shared_ptr<navpi_local_planner::ParamListener> m_param_listener;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr m_topic_interface;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_debug_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_gradient_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_edge_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_hole_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_raycast_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_vis_marker_pub;

  double m_last_angle;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_forward_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_angle_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_last_target_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_distance_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_sum_reward_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_dist_pub;

  int m_angular_samples;
  int m_window_size;
  std::string m_robot_frame;
  std::string m_map_frame;
  double m_min_z;
  double m_max_z;
  double m_sampling_distance;
  double m_forward_variance;
  double m_forward_weight;
  double m_angle_variance;
  double m_angle_weight;
  double m_last_target_variance;
  double m_last_target_weight;
  double m_neighborhood_arc;
  double m_distance_weight;
  double m_considered_distance;
  double m_max_gradient;
  double m_max_step_size;
  double m_push_distance;
  double m_resolution;


  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;


  navpi_local_planner::Params m_params;

  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_vdb_map;
};

} // namespace navpi


#endif // !NAVPI_OBSTACLE_AVOIDANCE_H
