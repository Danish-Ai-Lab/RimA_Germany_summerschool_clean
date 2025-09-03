set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}" ${CMAKE_MODULE_PATH})
include(CMakeFindDependencyMacro)
find_dependency(vdb_mapping)
include("${CMAKE_CURRENT_LIST_DIR}/planner_vdb_baseTargets.cmake")
