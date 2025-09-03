cmake_minimum_required(VERSION 3.14)
project(rumps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  INTERFACE
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

# Export package
ament_export_targets("export_${PROJECT_NAME}")
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 OpenMP)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_leafs test/test_leafs.cpp)
  target_link_libraries(test_leafs ${PROJECT_NAME})

  ament_add_gtest(test_navpi_issues test/test_navpi_issues.cpp)
  target_link_libraries(test_navpi_issues ${PROJECT_NAME})

  # build benchmarks
  find_package(benchmark REQUIRED)

  add_executable(benchmark_leafs test/benchmark_leafs.cpp)
  target_link_libraries(benchmark_leafs benchmark::benchmark ${PROJECT_NAME})
  install(TARGETS benchmark_leafs DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
