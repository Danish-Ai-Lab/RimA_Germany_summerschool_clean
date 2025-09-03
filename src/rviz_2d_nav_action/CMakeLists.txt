cmake_minimum_required(VERSION 3.8)
project(rviz_2d_nav_action_navpi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(Qt5 REQUIRED Core Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)

find_package(backward_ros REQUIRED)

find_package(navpi_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(nav_goal_tool_headers_to_moc
  include/rviz_2d_nav_action_navpi/nav_goal_tool.hpp
  )

foreach(header "${nav_goal_tool_headers_to_moc}")
  qt5_wrap_cpp(nav_goal_tool_moc_files "${header}")
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/nav_goal_tool.cpp
  ${nav_goal_tool_moc_files}
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "Qt5"
  "pluginlib"
  "rclcpp"
  "rviz_rendering"
  "rviz_default_plugins"
  "rviz_ogre_vendor"
  "rviz_common"
  "backward_ros"
  "navpi_interfaces"
  "tf2_geometry_msgs"
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "rviz_2d_nav_action_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
