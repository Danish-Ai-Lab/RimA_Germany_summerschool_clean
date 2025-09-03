#include "rclcpp/rclcpp.hpp"
#include "vdb_mapping/OccupancyVDBMapping.hpp"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"

#include "navpi/navpi_coordinator.hpp"
#include "navpi/navpi_global_planner.hpp"
#include "navpi/navpi_local_planner.hpp"
#include "navpi/navpi_rmp_local_planner.hpp"
#include "navpi/navpi_recovery.hpp"
#include "navpi/navpi_sampling_planner.hpp"

// test stuff

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/executor.hpp>

// end test stuff


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;

  options.arguments({"--ros-args", "-r", "__node:=vdb_mapping"});
  auto mapping = std::make_shared<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping>>(options);
  exec.add_node(mapping);


  options.arguments({"--ros-args", "-r", "__node:=navpi_recovery_behaviors"});
  auto recovery = std::make_shared<navpi::NavPiRecovery>(options);
  recovery->setMap(mapping->getMap());
  exec.add_node(recovery->get_node_base_interface());
  recovery->configure();
  recovery->activate();

  options.arguments({"--ros-args", "-r", "__node:=navpi_global_planner"});
  // auto global_planner = std::make_shared<navpi::NavPiGlobalPlanner>(options);
  auto global_planner = std::make_shared<navpi::NavPiSamplingPlanner>(options);
  global_planner->setMap(mapping->getMap());
  exec.add_node(global_planner->get_node_base_interface());
  global_planner->configure();
  global_planner->activate();

  // options.arguments({"--ros-args", "-r", "__node:=navpi_local_planner"});
  // auto local_planner = std::make_shared<navpi::NavPiLocalPlanner>(options);
  // local_planner->setMap(mapping->getMap());
  // exec.add_node(local_planner->get_node_base_interface());
  // local_planner->configure();
  // local_planner->activate();

  options.arguments({"--ros-args", "-r", "__node:=navpi_rmp_local_planner"});
  auto rmp_local_planner = std::make_shared<navpi::NavPiRMPLocalPlanner>(options);
  rmp_local_planner->setMap(mapping->getMap());
  exec.add_node(rmp_local_planner->get_node_base_interface());
  rmp_local_planner->configure();
  rmp_local_planner->activate();

  options.arguments({"--ros-args", "-r", "__node:=navpi_coordinator"});
  auto coordinator = std::make_shared<navpi::NavPiCoordinator>(options);
  exec.add_node(coordinator->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
