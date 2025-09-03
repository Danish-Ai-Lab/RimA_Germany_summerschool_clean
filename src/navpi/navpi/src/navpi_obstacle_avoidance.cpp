#include "navpi/navpi_obstacle_avoidance.hpp"
#include <chrono>

namespace navpi {

NavPiObstacleAvoidance::NavPiObstacleAvoidance(
  rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<navpi_local_planner::ParamListener> param_listener,
  std::shared_ptr<vdb_mapping::OccupancyVDBMapping> vdb_map,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topic_if)
  : m_clock(clock)
  , m_param_listener(param_listener)
  , m_tf_buffer(clock)
  , m_tf_listener(m_tf_buffer)
  , m_vdb_map(vdb_map)
  , m_topic_interface(topic_if)
{
  m_debug_pub = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
    m_topic_interface, "obstacle_points", 10);
  m_gradient_pub =
    rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_interface, "gradient_debug", 1);
  m_edge_pub =
    rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_interface, "edge_debug", 1);
  m_hole_pub =
    rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_interface, "hole_debug", 1);
  m_raycast_pub =
    rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_interface, "raycast_debug", 1);

  m_last_angle = 0;

  m_reward_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "reward_debug", 1);
  m_forward_reward_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "forward_reward", 1);
  m_angle_reward_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "angle_reward", 1);
  m_last_target_reward_pub = rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(
    m_topic_interface, "last_target_reward", 1);
  m_distance_reward_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "distance_reward", 1);
  m_sum_reward_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "sum_reward", 1);
  m_dist_pub =
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(m_topic_interface, "map_dist", 1);

  // m_vis_marker_pub = m_root_nh.advertise<visualization_msgs::Marker>("vis_marker", 1);
  m_vis_marker_pub =
    rclcpp::create_publisher<visualization_msgs::msg::Marker>(m_topic_interface, "vis_marker", 10);

  m_params = m_param_listener->get_params();

  // TODO params
  m_angular_samples      = 180;
  m_robot_frame          = m_params.robot_frame;
  m_map_frame            = m_params.map_frame;
  m_min_z                = -0.1;
  m_max_z                = 1.4;
  m_sampling_distance    = 3;
  m_window_size          = 2;
  m_forward_variance     = M_PI / 2.0;
  m_forward_weight       = 0.1;
  m_angle_variance       = M_PI / 2.0;
  m_angle_weight         = 0.4;
  m_last_target_variance = M_PI / 2.0;
  m_last_target_weight   = 0.2;
  m_distance_weight      = 0.5;
  m_neighborhood_arc     = 1.0;
  m_considered_distance  = 5.0;
  m_max_gradient         = 0.3;
  m_max_step_size        = 0.05;
  m_push_distance        = 0.5;

  std::shared_lock map_lock(*m_vdb_map->getMapMutex());
  m_resolution = m_vdb_map->getGrid()->transform().voxelSize()[0];
  map_lock.unlock();
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
NavPiObstacleAvoidance::adjustTarget(geometry_msgs::msg::PoseStamped& next_pose,
                                     geometry_msgs::msg::PoseStamped& final_pose)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>());

  double dist_to_target =
    std::sqrt(std::pow(next_pose.pose.position.x - final_pose.pose.position.x, 2) +
              std::pow(next_pose.pose.position.y - final_pose.pose.position.y, 2));

  rclcpp::Time stamp = m_clock->now();
  geometry_msgs::msg::TransformStamped base_to_map_tf;
  geometry_msgs::msg::TransformStamped map_to_base_tf;
  try
  {
    base_to_map_tf = m_tf_buffer.lookupTransform(m_map_frame, m_robot_frame, tf2::TimePointZero);
    map_to_base_tf = m_tf_buffer.lookupTransform(m_robot_frame, m_map_frame, tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Transform from base to map frame failed: " << ex.what() << std::endl;
    return obstacles;
  }

  // std::cout << "tf took " << (ros::Time::now()-a).toSec() << "s" << std::endl;
  sensor_msgs::msg::LaserScan reward =
    extractNavigationCorridor(base_to_map_tf, map_to_base_tf, obstacles);
  if (reward.ranges.empty())
  {
    std::cout << "Navigation Corridor was invalid" << std::endl;
    return obstacles;
  }

  if (dist_to_target < 0.25)
  {
    return obstacles;
  }

  m_dist_pub->publish(reward);

  calculateRewards(reward, next_pose, base_to_map_tf, map_to_base_tf);

  double new_angle = getNewTargetAngle(reward);
  m_last_angle     = new_angle;


  Eigen::Matrix<double, 4, 1> next_pose_eigen;
  next_pose_eigen << next_pose.pose.position.x, next_pose.pose.position.y,
    next_pose.pose.position.z, 1.0;
  next_pose_eigen = tf2::transformToEigen(map_to_base_tf).matrix() * next_pose_eigen;

  // TODO Use the sampling distance of the path sampler
  double dist_to_next_pose;
  dist_to_next_pose = std::min(1.0, next_pose_eigen.head(2).norm());


  Eigen::Vector4d point_eigen(cos(new_angle) * dist_to_next_pose,
                              sin(new_angle) * dist_to_next_pose,
                              next_pose.pose.position.z,
                              1);

  Eigen::Matrix<double, 4, 1> force_vector;
  Eigen::Matrix<double, 4, 1> diff_vector;
  Eigen::Matrix<double, 4, 1> p_buf;
  force_vector << 0, 0, 0, 0;
  bool force_available = false;
  p_buf << point_eigen.x(), point_eigen.y(), 0, 0;
  double max_length = 0.0;
  for (auto& p : obstacles->points)
  {
    Eigen::Matrix<double, 4, 1> temp_point;
    temp_point << p.x, p.y, 0, 0;

    diff_vector = p_buf - temp_point;
    if (diff_vector.norm() < m_push_distance)
    {
      force_available = true;
      // Each obstacle contributes to the result direction weighted by the inverse of its distance
      force_vector += (1.0 / diff_vector.norm()) * diff_vector;

      // We want to estimate the push length based on the closest obstacle such that
      // it is pushed away to m_push_distance away from the obstacle.
      // Obviously, this is not 100% correct, as the push direction is not the direction
      // towards the closest obstacle, but it should give a hint.
      if ((m_push_distance - diff_vector.norm()) > max_length)
      {
        max_length = (m_push_distance - diff_vector.norm());
      }
    }
  }
  // normalize the resulting push distance as explained above.
  force_vector = max_length * force_vector.normalized();

  if (force_available)
  {
    point_eigen += force_vector;
  }

  point_eigen = tf2::transformToEigen(base_to_map_tf).matrix() * point_eigen;

  next_pose.pose.position.x = point_eigen.x();
  next_pose.pose.position.y = point_eigen.y();
  next_pose.pose.position.z = next_pose.pose.position.z;

  m_reward_pub->publish(reward);
  return obstacles;
}


sensor_msgs::msg::LaserScan NavPiObstacleAvoidance::extractNavigationCorridor(
  geometry_msgs::msg::TransformStamped& base_to_map_tf,
  geometry_msgs::msg::TransformStamped& map_to_base_tf,
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gradient_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr raycast_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix<double, Eigen::Dynamic, 1> distance_rewards(m_angular_samples);

  Eigen::Affine3d eigen_map_to_base_tf = tf2::transformToEigen(map_to_base_tf);


  openvdb::FloatGrid::Ptr grid = m_vdb_map->getMapSectionGrid(
    Eigen::Matrix<double, 3, 1>(-m_sampling_distance, -m_sampling_distance, -m_sampling_distance),
    Eigen::Matrix<double, 3, 1>(m_sampling_distance, m_sampling_distance, m_sampling_distance),
    eigen_map_to_base_tf.inverse().matrix());

  openvdb::tools::dilateActiveValues(grid->tree(), 3, openvdb::tools::NN_FACE_EDGE_VERTEX);
  openvdb::tools::erodeActiveValues(
    grid->tree(), 3, openvdb::tools::NN_FACE_EDGE_VERTEX, openvdb::tools::EXPAND_TILES, true);


  visualization_msgs::msg::Marker mark;
  sensor_msgs::msg::PointCloud2 peter;
  nav_msgs::msg::OccupancyGrid gert;

  VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::createMappingOutput(
    grid, m_map_frame, mark, peter, gert, true, false, false); //, 0, 0);

  m_vis_marker_pub->publish(mark);

  openvdb::FloatGrid::Accessor acc = grid->getAccessor();

  // set sampling angular resolution
  double angular_resolution = 2 * M_PI / m_angular_samples;

  int sampling_index_distance = (int)(m_sampling_distance / grid->transform().voxelSize()[0]);

  openvdb::Vec3d start;
  RayT ray;


  tf2::Quaternion q(base_to_map_tf.transform.rotation.x,
                    base_to_map_tf.transform.rotation.y,
                    base_to_map_tf.transform.rotation.z,
                    base_to_map_tf.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);

  sensor_msgs::msg::LaserScan reward;
  reward.angle_min       = -M_PI;
  reward.angle_max       = M_PI;
  reward.angle_increment = angular_resolution;
  reward.range_min       = 0.01;
  reward.range_max       = m_sampling_distance + 1;
  reward.header.frame_id = m_robot_frame;
  reward.header.stamp    = map_to_base_tf.header.stamp;
  reward.ranges          = std::vector<float>(m_angular_samples, 0.0);
  reward.intensities     = std::vector<float>(m_angular_samples, 1.0);

  // for one vertical line:
  for (size_t i = 0; i < m_angular_samples; ++i)
  {
    geometry_msgs::msg::Vector3Stamped direction_vector;
    direction_vector.header.frame_id = m_robot_frame;
    direction_vector.header.stamp    = base_to_map_tf.header.stamp;
    double current_angle             = -M_PI + i * angular_resolution;

    direction_vector.vector.x = cos(current_angle);
    direction_vector.vector.y = sin(current_angle);
    direction_vector.vector.z = 0;

    try
    {
      direction_vector = m_tf_buffer.transform(direction_vector, m_map_frame);
    }
    catch (tf2::TransformException& ex)
    {
      std::cout << "Transform of direction vector failed" << ex.what() << std::endl;
      return sensor_msgs::msg::LaserScan();
    }

    openvdb::Vec3d dir(
      direction_vector.vector.x, direction_vector.vector.y, direction_vector.vector.z);

    ray.setDir(grid->worldToIndex(dir));

    // raycast points between max and min range in base_link
    std::vector<Eigen::Vector3d> segment_points;
    start = openvdb::Vec3d(base_to_map_tf.transform.translation.x,
                           base_to_map_tf.transform.translation.y,
                           base_to_map_tf.transform.translation.z);


    RayT::Vec3Type start_index = grid->worldToIndex(start);
    ray.setEye(start_index);

    DDAT dda;
    dda.init(ray);

    int counter = 0;


    int max_tries = 20;
    int max_h     = 10;
    int min_h     = 10;


    openvdb::Coord xyz, last_xyz, mod;
    last_xyz = dda.voxel();
    mod      = openvdb::Coord(0, 0, 0);

    std::vector<Eigen::Matrix<double, 3, 1>> floor_profile;
    std::vector<double> floor_height;

    double outer_radius = 1.5 / m_resolution;

    double traversed_distance = 0;
    while (traversed_distance < sampling_index_distance)
    {
      dda.step();

      // find bottom element
      xyz                = dda.voxel();
      traversed_distance = std::sqrt(std::pow((start_index.x() - xyz.x()), 2) +
                                     std::pow((start_index.y() - xyz.y()), 2));

      xyz.z() = last_xyz.z();

      bool found_ground = false;

      for (size_t j = 0; j < max_tries; ++j)
      {
        mod.z() = j;
        if (acc.isValueOn(xyz + mod) ||
            ((acc.getValue(xyz + mod) > -0.1) && traversed_distance > outer_radius))
        {
          found_ground = true;
          xyz += mod;
          break;
        }
        if (acc.isValueOn(xyz - mod) ||
            ((acc.getValue(xyz - mod) > -0.1) && traversed_distance > outer_radius))
        {
          found_ground = true;
          xyz -= mod;
          break;
        }
      }

      // Go up until a free space is reached
      while (acc.isValueOn(xyz))
      {
        xyz.z()++;
      }

      // Check upper bounds
      // TODO rework
      int robot_height_index   = 20;
      int robot_height_counter = 0;

      mod = openvdb::Coord(0, 0, 0);
      while (robot_height_counter < robot_height_index)
      {
        if (acc.isValueOn(xyz + mod))
        {
          found_ground = false;
          break;
        }
        else
        {
          robot_height_counter++;
          mod.z() = robot_height_counter;
        }
      }


      // Remember last value... basically i could just use the z value maybe TODO
      last_xyz = xyz;


      if (found_ground)
      {
        openvdb::Vec3d point = grid->indexToWorld(xyz);
        floor_profile.push_back(Eigen::Matrix<double, 3, 1>(point.x(), point.y(), point.z()));
        floor_height.push_back(point.z());
        raycast_cloud->points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
      }
      else
      {
        break;
      }
      counter++;
    }


    if (floor_profile.size() >= 1)
    {
      floor_height.insert(floor_height.begin(), floor_height[0]);
      floor_height.insert(floor_height.begin(), floor_height[0]);
      floor_height.push_back(floor_height.back());
      floor_height.push_back(floor_height.back());
      std::vector<double> filtered;

      // Remove effects of discretization for gradient calculation
      for (size_t j = 2; j < floor_height.size() - 2; ++j)
      {
        double buffer;
        buffer = 0.1 * (floor_height[j - 2] + 2 * floor_height[j - 1] + 4 * floor_height[j] +
                        2 * floor_height[j + 1] + floor_height[j + 2]);
        filtered.push_back(buffer);
      }

      double step_size;
      double gradient;

      int first, second;
      pcl::PointXYZI buf;

      int filter_size = 10;

      for (int j = 1; j < floor_profile.size(); ++j)
      {
        step_size = floor_profile[j].z() - floor_profile[j - 1].z();

        first  = std::max(0, j - filter_size);
        second = std::min((int)filtered.size() - 1, j + filter_size);

        // gradient = (filtered[second]-filtered[first])/(second-first)/m_resolution;
        gradient = (filtered[second] - filtered[first]) /
                   std::sqrt(std::pow(floor_profile[second].x() - floor_profile[first].x(), 2) +
                             std::pow(floor_profile[second].y() - floor_profile[first].y(), 2));

        buf.x         = floor_profile[j - 1].x();
        buf.y         = floor_profile[j - 1].y();
        buf.z         = filtered[j - 1];
        buf.intensity = 1;

        if (std::fabs(gradient) > m_max_gradient)
        {
          break;
        }
        if (std::fabs(step_size) > m_max_step_size)
        {
          break;
        }
      }
      cloud->points.push_back(buf);
      obstacles->points.push_back(pcl::PointXYZ(buf.x, buf.y, buf.z));


      Eigen::Matrix<double, 4, 1> point;

      point << buf.x, buf.y, buf.z, 1;
      point            = eigen_map_to_base_tf.matrix() * point;
      reward.ranges[i] = point.head(2).norm();
    }
  }
  raycast_cloud->width  = raycast_cloud->points.size();
  raycast_cloud->height = 1;
  pcl::transformPointCloud(*raycast_cloud, *raycast_cloud, eigen_map_to_base_tf);


  cloud->width  = cloud->points.size();
  cloud->height = 1;
  pcl::transformPointCloud(*cloud, *cloud, eigen_map_to_base_tf);
  obstacles->width  = obstacles->points.size();
  obstacles->height = 1;
  pcl::transformPointCloud(*obstacles, *obstacles, eigen_map_to_base_tf);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = m_robot_frame;
  cloud_msg.header.stamp    = map_to_base_tf.header.stamp;
  m_debug_pub->publish(cloud_msg);

  sensor_msgs::msg::PointCloud2 raycast_cloud_msg;
  pcl::toROSMsg(*raycast_cloud, raycast_cloud_msg);
  raycast_cloud_msg.header.frame_id = m_robot_frame;
  raycast_cloud_msg.header.stamp    = map_to_base_tf.header.stamp;
  m_raycast_pub->publish(raycast_cloud_msg);


  return reward;
}

float NavPiObstacleAvoidance::calculateDistanceFromSegment(
  const std::vector<Eigen::Vector3d>& segment_points)
{
  double max_dist = m_sampling_distance;
  bool all_free   = true;
  for (int j = 0; j < segment_points.size(); ++j)
  {
    if (segment_points[j] != Eigen::Vector3d(0, 0, 0))
    {
      all_free = false;
      break;
    }
  }
  if (all_free)
  {
    return max_dist;
  }

  for (int j = 0; j < segment_points.size(); ++j)
  {
    // if current point is a max range point
    if (segment_points[j] == Eigen::Vector3d(0, 0, 0))
    {
      max_dist = std::min(max_dist, m_sampling_distance);
      continue;
    }
    else
    {
      // Calculate segment window
      std::vector<Eigen::Vector3d> normal_window;
      for (int k = std::max(0, (j - m_window_size)); k <= j + m_window_size; ++k)
      {
        if (segment_points[k] != Eigen::Vector3d(0, 0, 0))
        {
          normal_window.push_back(segment_points[k]);
        }
      }

      // if window is too small
      if (normal_window.size() < 2)
      {
        max_dist = std::min(max_dist, m_sampling_distance);
        continue;
      }
      else
      {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> regression = line_fitting(normal_window);
        double dist =
          std::sqrt(std::pow(regression.second.x(), 2) + std::pow(regression.second.y(), 2));

        // If the gradient is inf
        if (dist == 0)
        {
          max_dist = std::min(max_dist, m_sampling_distance);
          continue;
        }
        else
        {
          double gradient = regression.second.z() / dist;
          // TODO REMOVE MAGIC VALUE
          if (std::fabs(gradient) < m_max_gradient)
          {
            max_dist = std::max(
              max_dist,
              std::sqrt(std::pow(segment_points[j].x(), 2) + std::pow(segment_points[j].y(), 2)));
          }
          else
          {
            max_dist = std::min(
              max_dist,
              std::sqrt(std::pow(segment_points[j].x(), 2) + std::pow(segment_points[j].y(), 2)));
          }
        }
      }
    }
  }
  return max_dist;
}

void NavPiObstacleAvoidance::calculateRewards(sensor_msgs::msg::LaserScan& scan,
                                              geometry_msgs::msg::PoseStamped& target,
                                              geometry_msgs::msg::TransformStamped& base_to_map_tf,
                                              geometry_msgs::msg::TransformStamped& map_to_base_tf)
{
  Eigen::Vector4d point_eigen(
    target.pose.position.x, target.pose.position.y, target.pose.position.z, 1);
  point_eigen         = tf2::transformToEigen(map_to_base_tf).matrix() * point_eigen;
  double target_angle = std::atan2(point_eigen.y(), point_eigen.x());


  sensor_msgs::msg::LaserScan forward_reward_scan;
  sensor_msgs::msg::LaserScan angle_reward_scan;
  sensor_msgs::msg::LaserScan distance_reward_scan;
  sensor_msgs::msg::LaserScan last_target_reward_scan;
  sensor_msgs::msg::LaserScan sum_reward_scan;

  forward_reward_scan                 = scan;
  forward_reward_scan.ranges          = std::vector<float>(scan.ranges.size(), 0.55);
  forward_reward_scan.intensities     = std::vector<float>();
  angle_reward_scan                   = scan;
  angle_reward_scan.ranges            = std::vector<float>(scan.ranges.size(), 0.65);
  angle_reward_scan.intensities       = std::vector<float>();
  distance_reward_scan                = scan;
  distance_reward_scan.ranges         = std::vector<float>(scan.ranges.size(), 0.75);
  distance_reward_scan.intensities    = std::vector<float>();
  last_target_reward_scan             = scan;
  last_target_reward_scan.ranges      = std::vector<float>(scan.ranges.size(), 0.80);
  last_target_reward_scan.intensities = std::vector<float>();
  sum_reward_scan                     = scan;
  sum_reward_scan.ranges              = std::vector<float>(scan.ranges.size(), 0.85);
  sum_reward_scan.intensities         = std::vector<float>();

  sensor_msgs::msg::LaserScan adjusted_scan;
  adjusted_scan = scan;

  for (int i = 0; i < scan.ranges.size(); ++i)
  {
    if (scan.ranges[i] > 0)
    {
      int considered_neighbors =
        (int)((m_neighborhood_arc / scan.ranges[i]) / scan.angle_increment);
      considered_neighbors = std::min((int)(m_angular_samples / 2 - 1), considered_neighbors);

      for (int j = 1; j <= considered_neighbors; j++)
      {
        int upper_neighbor = i + j;
        int lower_neighbor = i - j;

        upper_neighbor -= (upper_neighbor >= scan.ranges.size()) ? scan.ranges.size() : 0;
        lower_neighbor += (lower_neighbor < 0) ? scan.ranges.size() : 0;

        adjusted_scan.ranges[upper_neighbor] =
          std::min(adjusted_scan.ranges[upper_neighbor], scan.ranges[i]);
        adjusted_scan.ranges[lower_neighbor] =
          std::min(adjusted_scan.ranges[lower_neighbor], scan.ranges[i]);
      }
    }
  }

  scan = adjusted_scan;

  std::vector<float> intensities;
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    double current_angle = scan.angle_min + scan.angle_increment * i;

    double forward_reward = exp(-0.5 * std::pow((current_angle - 0.0) / m_forward_variance, 2));
    forward_reward_scan.intensities.push_back(forward_reward);

    double angle_to_target = (current_angle - target_angle);
    angle_to_target += (angle_to_target < -M_PI)  ? 2 * M_PI
                       : (angle_to_target > M_PI) ? -2 * M_PI
                                                  : 0;

    double angle_reward = exp(-0.5 * std::pow(angle_to_target / m_angle_variance, 2));

    angle_reward_scan.intensities.push_back(angle_reward);

    double last_target_reward =
      exp(-0.5 * std::pow((current_angle - m_last_angle) / m_last_target_variance, 2));
    last_target_reward_scan.intensities.push_back(last_target_reward);

    double range_min = scan.ranges[i];

    double distance_reward = (1.0 / m_considered_distance) * (range_min);
    if (distance_reward > 1)
    {
      distance_reward = 1;
    }

    distance_reward_scan.intensities.push_back(distance_reward);

    double reward_sum = (m_angle_weight * angle_reward) + (m_distance_weight * distance_reward) +
                        (m_forward_weight * forward_reward) +
                        (m_last_target_weight * last_target_reward);

    sum_reward_scan.intensities.push_back(reward_sum);
    intensities.push_back(reward_sum);
  }

  m_forward_reward_pub->publish(forward_reward_scan);
  m_angle_reward_pub->publish(angle_reward_scan);
  m_distance_reward_pub->publish(distance_reward_scan);
  m_last_target_reward_pub->publish(last_target_reward_scan);
  m_sum_reward_pub->publish(sum_reward_scan);


  scan.intensities = intensities;
}

double NavPiObstacleAvoidance::getNewTargetAngle(const sensor_msgs::msg::LaserScan& scan)
{
  double max_reward = 0;
  int max_index     = 0;
  for (size_t i = 0; i < scan.intensities.size(); ++i)
  {
    if (scan.intensities[i] > max_reward)
    {
      max_reward = scan.intensities[i];
      max_index  = i;
    }
  }
  double angle = scan.angle_min + scan.angle_increment * max_index;
  return angle;
}


std::pair<Eigen::Vector3d, Eigen::Vector3d>
NavPiObstacleAvoidance::line_fitting(const std::vector<Eigen::Vector3d>& points)
{
  size_t num_points = points.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> centers(num_points, 3);
  for (size_t i = 0; i < num_points; ++i)
  {
    centers.row(i) = points[i];
  }

  Eigen::Vector3d origin   = centers.colwise().mean();
  Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
  Eigen::MatrixXd cov      = centered.adjoint() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
  Eigen::Vector3d axis = eig.eigenvectors().col(2).normalized();
  return std::make_pair(origin, axis);
}


} // namespace navpi
