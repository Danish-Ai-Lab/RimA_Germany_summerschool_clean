#ifndef NAVPI_PATH_SAMPLER_H
#define NAVPI_PATH_SAMPLER_H


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <cmath>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>

#include <navpi/navpi_path_sampler_parameters.hpp>
#include "vdb_mapping/OccupancyVDBMapping.hpp"

namespace navpi {

class NavPiPathSampler
{
public:
  NavPiPathSampler(rclcpp::Clock::SharedPtr clock,
                   std::shared_ptr<navpi_path_sampler::ParamListener> param_listener,
                   std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
                   rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if);
  NavPiPathSampler(NavPiPathSampler&&)                 = default;
  NavPiPathSampler(const NavPiPathSampler&)            = default;
  NavPiPathSampler& operator=(NavPiPathSampler&&)      = default;
  NavPiPathSampler& operator=(const NavPiPathSampler&) = default;
  ~NavPiPathSampler()                                  = default;

  void setPath(const nav_msgs::msg::Path& path);
  geometry_msgs::msg::PoseStamped
  sampleNextPoseInPath(const geometry_msgs::msg::PoseStamped& current_pose);

private:
  nav_msgs::msg::Path m_path;
  bool m_use_adaptive_sampling;

  unsigned int m_path_size;
  unsigned int m_chain_size;
  unsigned int m_current_segment_id;
  double m_chain_angle_difference;
  double m_sample_distance;


  unsigned int findNearestSegment(const geometry_msgs::msg::PoseStamped& pose);
  unsigned int calculateChainSize();
  double calculateDistanceToSegment(const geometry_msgs::msg::PoseStamped& pose,
                                    const unsigned int segment_id);
  std::shared_ptr<navpi_path_sampler::ParamListener> m_param_listener;

  tf2::Stamped<tf2::Transform> poseToTransform(const geometry_msgs::msg::PoseStamped& pose_in);
};

} // namespace navpi

#endif /* NAVPI_PATH_SAMPLER_H */
