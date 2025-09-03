cmake_minimum_required(VERSION 3.8)
project(navpi)


if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(astar_vdb REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navpi_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vdb_mapping_ros2 REQUIRED)
find_package(backward_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ompl REQUIRED)
find_package(rumps REQUIRED)

generate_parameter_library(
  navpi_local_planner_parameters # cmake target name for the parameter library
  config/navpi_local_planner_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_rmp_local_planner_parameters # cmake target name for the parameter library
  config/navpi_rmp_local_planner_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_global_planner_parameters # cmake target name for the parameter library
  config/navpi_global_planner_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_coordinator_parameters # cmake target name for the parameter library
  config/navpi_coordinator_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_recovery_parameters # cmake target name for the parameter library
  config/navpi_recovery_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_path_sampler_parameters # cmake target name for the parameter library
  config/navpi_path_sampler_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  navpi_diff_drive_controller_parameters # cmake target name for the parameter library
  config/navpi_diff_drive_controller_parameters.yaml # path to input yaml file
)

include_directories(include)
include_directories(SYSTEM ${OMPL_INCLUDE_DIRS})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_toolbox
  backward_ros
  geometry_msgs
  nav_msgs
  navpi_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  vdb_mapping_ros2
  visualization_msgs
  ompl
  rumps
)

add_library(navpi_global_planner_component SHARED
  src/navpi_global_planner.cpp
)
target_link_libraries(navpi_global_planner_component
  astar_vdb::astar_vdb
  navpi_global_planner_parameters
)
target_compile_features(navpi_global_planner_component PUBLIC cxx_std_17)
ament_target_dependencies(navpi_global_planner_component
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rclcpp_components_register_nodes(navpi_global_planner_component "navpi::NavPiGlobalPlanner")

add_library(navpi_sampling_planner_component SHARED
  src/navpi_sampling_planner.cpp
  src/IntegerMotionValidator.cpp
  src/IntegerStateSampler.cpp
  src/IntegerStateSpace.cpp
  src/MapHandle.cpp
  src/RRTstarConnect.cpp
)
target_link_libraries(navpi_sampling_planner_component
  navpi_global_planner_parameters
  ${OMPL_LIBRARIES}
)
target_compile_features(navpi_sampling_planner_component PUBLIC cxx_std_17)
ament_target_dependencies(navpi_sampling_planner_component
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rclcpp_components_register_nodes(navpi_sampling_planner_component "navpi::NavPiSamplingPlanner")


add_library(navpi_local_planner_component SHARED
  src/navpi_local_planner.cpp
  src/navpi_path_sampler.cpp
  src/navpi_obstacle_avoidance.cpp
  src/navpi_base_controller.cpp
  src/navpi_safety.cpp
  src/utils.cpp
)
target_compile_features(navpi_local_planner_component PUBLIC cxx_std_17)
ament_target_dependencies(navpi_local_planner_component
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(navpi_local_planner_component
  navpi_local_planner_parameters
  navpi_path_sampler_parameters
)
rclcpp_components_register_nodes(navpi_local_planner_component "navpi::NavPiLocalPlanner")

add_library(navpi_rmp_local_planner_component SHARED
  src/navpi_rmp_local_planner.cpp
  src/navpi_path_sampler.cpp
  src/navpi_diff_drive_controller.cpp
  src/utils.cpp
)
target_compile_features(navpi_rmp_local_planner_component PUBLIC cxx_std_17)
ament_target_dependencies(navpi_rmp_local_planner_component
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(navpi_rmp_local_planner_component
  navpi_rmp_local_planner_parameters
  navpi_diff_drive_controller_parameters
  navpi_path_sampler_parameters
)
rclcpp_components_register_nodes(navpi_rmp_local_planner_component "navpi::NavPiRMPLocalPlanner")

add_library(navpi_recovery_component SHARED
  src/navpi_recovery.cpp
)
target_link_libraries(navpi_recovery_component
  astar_vdb::astar_vdb
  navpi_recovery_parameters
)
target_compile_features(navpi_recovery_component PUBLIC cxx_std_17)
ament_target_dependencies(navpi_recovery_component
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rclcpp_components_register_nodes(navpi_recovery_component "navpi::NavPiRecovery")



add_library(${PROJECT_NAME}_coordinator SHARED
  src/navpi_coordinator.cpp
)
target_link_libraries(${PROJECT_NAME}_coordinator
  navpi_coordinator_parameters
)
target_compile_features(${PROJECT_NAME}_coordinator PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_coordinator
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rclcpp_components_register_nodes(${PROJECT_NAME}_coordinator "navpi::NavPiCoordinator")






add_executable(${PROJECT_NAME}_node
  src/navpi_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  navpi_global_planner_component
  navpi_sampling_planner_component
  navpi_local_planner_component
  navpi_rmp_local_planner_component
  navpi_recovery_component
  navpi_coordinator
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navpi_global_planner_component
  DESTINATION lib
)

install(TARGETS
  navpi_sampling_planner_component
  DESTINATION lib
)

install(TARGETS
  navpi_local_planner_component
  DESTINATION lib
)

install(TARGETS
  navpi_recovery_component
  DESTINATION lib
)

install(TARGETS
  ${PROJECT_NAME}_coordinator
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_global_planner_test test/global_planner_test.cpp)
  set_tests_properties(${PROJECT_NAME}_global_planner_test PROPERTIES
    ENVIRONMENT DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/resources)

  target_include_directories (${PROJECT_NAME}_global_planner_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_global_planner_test
    navpi_global_planner_component
  )
endif()

ament_package()
