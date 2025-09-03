set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}" ${CMAKE_MODULE_PATH})
include(CMakeFindDependencyMacro)
find_dependency(planner_vdb_base)
include("${CMAKE_CURRENT_LIST_DIR}/astar_vdbTargets.cmake")
