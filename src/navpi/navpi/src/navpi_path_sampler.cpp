#include "navpi/navpi_path_sampler.hpp"


namespace navpi {


NavPiPathSampler::NavPiPathSampler(
  rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<navpi_path_sampler::ParamListener> param_listener,
  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if)
  : m_param_listener(param_listener)
{
  // Do stuff
  m_current_segment_id = 0;
  m_chain_size         = 0;
  m_path_size          = 0;
  m_path               = nav_msgs::msg::Path();
}

void NavPiPathSampler::setPath(const nav_msgs::msg::Path& path)
{
  auto params = m_param_listener->get_params();

  this->m_chain_angle_difference = params.chain_angle_difference;
  this->m_sample_distance        = params.sample_distance;
  this->m_use_adaptive_sampling  = params.use_adaptive_sampling;

  m_path      = path;
  m_path_size = m_path.poses.size();
  std::cout << " Path size: " << m_path_size << std::endl;
  m_chain_size = calculateChainSize();
  std::cout << " Chain size: " << m_chain_size << std::endl;
  m_current_segment_id = 0;
}

geometry_msgs::msg::PoseStamped
NavPiPathSampler::sampleNextPoseInPath(const geometry_msgs::msg::PoseStamped& current_pose)
{
  if (m_path_size == 2)
  {
    // This is in case a direct goal without path planning is send to the local planner.
    return m_path.poses.back();
  }
  if (m_use_adaptive_sampling)
  {
    unsigned int current_index = std::min(findNearestSegment(current_pose), m_path_size - 1);

    double angle_diff          = 0;
    unsigned int current_chain = 0;

    tf2::Stamped<tf2::Transform> nearest_segment;
    nearest_segment = poseToTransform(m_path.poses[current_index]);

    while (angle_diff < m_chain_angle_difference && current_chain < m_chain_size &&
           current_index + current_chain < m_path_size - 1)
    {
      double roll, pitch, yaw;
      tf2::Stamped<tf2::Transform> lookup_segment;
      lookup_segment        = poseToTransform(m_path.poses[current_index + current_chain + 1]);
      tf2::Transform result = nearest_segment.inverseTimes(lookup_segment);
      result.getBasis().getRPY(roll, pitch, yaw);
      angle_diff = std::abs(yaw);
      current_chain++;
    }
    // RCLCPP_DEBUG(this->get_logger(),
    //"Current index: %d; m_chain_size: %d",
    // findNearestSegment(current_pose),
    // m_chain_size);
    return m_path.poses[current_index + current_chain];
  }

  // RCLCPP_DEBUG(this->get_logger(),
  //"Current index: %d; m_chain_size: %d",
  // findNearestSegment(current_pose),
  // m_chain_size);
  return m_path.poses[std::min(findNearestSegment(current_pose) + m_chain_size, m_path_size - 1)];
}

unsigned int NavPiPathSampler::findNearestSegment(const geometry_msgs::msg::PoseStamped& pose)
{
  // Calculate segment nearest to robot at the last interval
  unsigned int index       = std::max<unsigned int>(m_current_segment_id, 1);
  double distance_to_index = calculateDistanceToSegment(pose, index);

  while (distance_to_index > 0 && index < m_path_size - 1)
  {
    index++;
    distance_to_index = calculateDistanceToSegment(pose, index);
  }
  m_current_segment_id = index;
  return index;
}
unsigned int NavPiPathSampler::calculateChainSize()
{
  double chain_length = 0;
  unsigned int index  = 0;

  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::PoseStamped next_pose;

  // RCLCPP_DEBUG(this->get_logger(), "Path length: %lu", path.poses.size());

  while (chain_length < m_sample_distance && index < m_path.poses.size() - 2)
  {
    current_pose = m_path.poses[index];
    next_pose    = m_path.poses[index + 1];

    auto x_distance = std::pow((current_pose.pose.position.x - next_pose.pose.position.x), 2);
    auto y_distance = std::pow((current_pose.pose.position.y - next_pose.pose.position.y), 2);
    auto z_distance = std::pow((current_pose.pose.position.z - next_pose.pose.position.z), 2);

    // RCLCPP_DEBUG(this->get_logger(),
    //"Index: %u x_dist: %f, y_dist: %f, z_dist: %f",
    // index,
    // x_distance,
    // y_distance,
    // z_distance);
    chain_length += std::sqrt(x_distance + y_distance + z_distance);
    index++;
  }
  return index;
}

double NavPiPathSampler::calculateDistanceToSegment(const geometry_msgs::msg::PoseStamped& pose,
                                                    const unsigned int segment_id)
{
  // TODO Adjust for 3d and tf2
  tf2::Stamped<tf2::Transform> path_segment;
  path_segment = poseToTransform(m_path.poses[segment_id]);

  double roll;
  double pitch;
  double yaw;
  path_segment.getBasis().getRPY(roll, pitch, yaw);

  return cos(yaw) * (pose.pose.position.x - path_segment.getOrigin().x()) +
         sin(yaw) * (pose.pose.position.y - path_segment.getOrigin().y());
}

tf2::Stamped<tf2::Transform>
NavPiPathSampler::poseToTransform(const geometry_msgs::msg::PoseStamped& pose_in)
{
  auto transform_out = tf2::Stamped<tf2::Transform>();
  // transform_out.stamp_ = pose_in.header.stamp;
  transform_out.frame_id_ = pose_in.header.frame_id;
  tf2::Transform temp;
  temp.setOrigin(
    tf2::Vector3(pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z));
  temp.setRotation(tf2::Quaternion(pose_in.pose.orientation.x,
                                   pose_in.pose.orientation.y,
                                   pose_in.pose.orientation.z,
                                   pose_in.pose.orientation.w));
  transform_out.setData(temp);
  return transform_out;
}


} // namespace navpi
