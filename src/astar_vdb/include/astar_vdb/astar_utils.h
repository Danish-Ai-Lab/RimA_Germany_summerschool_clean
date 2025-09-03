#pragma once

#include <planner_vdb_base/DataNode.h>

#include <iomanip>
#include <iostream>
#include <limits>
#include <openvdb/openvdb.h>
#include <sstream>
#include <string>

namespace astar_vdb {

struct AStarDataNode
{
  double f = std::numeric_limits<double>::infinity();
  double g = std::numeric_limits<double>::infinity();
  double h = std::numeric_limits<double>::infinity();
  openvdb::Coord parent;
  bool is_closed = false;
  bool is_open   = false;
};

using AStarValueType = DataNode<AStarDataNode>;
using AStarGridT     = openvdb::Grid<typename openvdb::tree::Tree4<AStarValueType, 5, 4, 3>::Type>;

using ValueType  = float;
using ValueGridT = openvdb::Grid<typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type>;

} // namespace astar_vdb
