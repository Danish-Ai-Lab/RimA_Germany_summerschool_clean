#include "navpi/navpi_global_planner.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include <geometry_msgs/msg/detail/point_stamped__traits.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <gtest/gtest.h>

TEST(navpi, SetupPlanner)
{
  rclcpp::NodeOptions options;
  double resolution   = 0.05;
  auto mapping        = std::make_shared<vdb_mapping::OccupancyVDBMapping>(resolution);
  auto global_planner = std::make_shared<navpi::NavPiGlobalPlanner>(options);
  global_planner->configure();
  // Map not set, should not be able to configure
  EXPECT_FALSE(global_planner->isInitialized());

  global_planner->setMap(mapping);
  global_planner->configure();

  EXPECT_TRUE(global_planner->isInitialized());
}

TEST(navpi, PlanOnEmptyMap)
{
  rclcpp::NodeOptions options;
  double resolution   = 0.05;
  auto mapping        = std::make_shared<vdb_mapping::OccupancyVDBMapping>(resolution);
  auto global_planner = std::make_shared<navpi::NavPiGlobalPlanner>(options);
  global_planner->setMap(mapping);

  std::vector<geometry_msgs::msg::PoseStamped> path;
  double cost;
  std::string message;

  global_planner->makePlan(
    geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::PoseStamped(), 0.0, path, cost, message);
  EXPECT_EQ(path.size(), 1);

  geometry_msgs::msg::PoseStamped test_pose;
  test_pose.pose.orientation.w = 1.0;
  EXPECT_EQ(path[0], test_pose);
  test_pose.pose.orientation.w = 0.0;
  EXPECT_NE(path[0], test_pose);
}

TEST(navpi, PlanOnVDBExample)
{
  rclcpp::NodeOptions options;
  double resolution = 0.05;
  auto mapping      = std::make_shared<vdb_mapping::OccupancyVDBMapping>(resolution);
  // TODO load map
  std::string vdb_file = std::string(std::getenv("DATADIR")) + "/example.vdb";
  mapping->loadMap(vdb_file);
  auto global_planner = std::make_shared<navpi::NavPiGlobalPlanner>(options);
  global_planner->setMap(mapping);


  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x    = 1.0;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x    = 4.0;
  goal.pose.orientation.w = 1.0;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  double cost;
  std::string message;

  global_planner->makePlan(start, goal, 0.0, path, cost, message);
  EXPECT_GT(path.size(), 1);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
