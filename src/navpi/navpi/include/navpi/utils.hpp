#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>

double
getEuclidDistanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                              const geometry_msgs::msg::PoseStamped& pose2);

double getAngleBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                            const geometry_msgs::msg::PoseStamped& pose2);

double getSignedAngleBetweenVectorsXY(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
