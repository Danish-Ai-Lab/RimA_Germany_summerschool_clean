#include "navpi/utils.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

double
getEuclidDistanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                                                 const geometry_msgs::msg::PoseStamped& pose2)
{
  double distance_x = pose1.pose.position.x - pose2.pose.position.x;
  double distance_y = pose1.pose.position.y - pose2.pose.position.y;
  // float distance_z = pose1.pose.position.z - pose2.pose.position.z;

  return sqrt(distance_x * distance_x + distance_y * distance_y); // + distance_z * distance_z);
}

double getAngleBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                                               const geometry_msgs::msg::PoseStamped& pose2)
{
  tf2::Quaternion quat1(pose1.pose.orientation.x,
                        pose1.pose.orientation.y,
                        pose1.pose.orientation.z,
                        pose1.pose.orientation.w);
  tf2::Quaternion quat2(pose2.pose.orientation.x,
                        pose2.pose.orientation.y,
                        pose2.pose.orientation.z,
                        pose2.pose.orientation.w);

  double angle_between = quat1.angleShortestPath(quat2);
  return angle_between;
}

double getSignedAngleBetweenVectorsXY(const Eigen::Vector3d &v1,
                                      const Eigen::Vector3d &v2) {
  // WARNING only in 2d for now
  double angle = std::atan2(v2[1], v2[0]) - std::atan2(v1[1], v1[0]);
  // wrap angle to -pi, pi
  return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}
