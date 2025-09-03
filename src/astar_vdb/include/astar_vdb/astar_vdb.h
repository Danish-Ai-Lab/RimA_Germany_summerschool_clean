#pragma once

#include "astar_vdb/astar_utils.h"
#include "astar_vdb/priority_queue.h"
#include <openvdb/openvdb.h>
#include <planner_vdb_base/planner_vdb_base.h>


namespace astar_vdb {


class AstarVDB : public planner_vdb_base::PlannerVDBBase
{
public:
  AstarVDB(typename ValueGridT::Ptr value_grid,
           bool debug_mode               = false,
           int ground_seach_level        = 1,
           bool ground_cache_on          = true,
           int successor_variant         = 3,
           int m_successor_restriction   = 2,
           int stepheight                = 4,
           int dilate                    = 5,
           int slope_ground_range        = 2,
           float max_slope_angle_degrees = 45);

  planner_vdb_base::SearchState step() override;

  bool setStartAndGoal(const openvdb::Coord& start, const openvdb::Coord& goal) override;

  std::vector<openvdb::Coord> extractPath() override;

  // Grid to debug planner expandation
  openvdb::Int32Grid::Ptr getExpandGrid() { return m_expand_grid; };

  typename AStarGridT::Ptr getAstarGrid() { return m_astar_grid; };

  void printSearchStats() override;

  bool isPathAvailable() override;


private:
  std::shared_ptr<PriorityQueue> m_open_list;

  typename AStarGridT::Ptr m_astar_grid;
  std::shared_ptr<AStarGridT::Accessor> m_astar_grid_acc;


  openvdb::Int32Grid::Ptr m_expand_grid;
  std::shared_ptr<openvdb::Int32Grid::Accessor> m_expand_grid_acc;


  void initialize();

  openvdb::Coord m_search_start;
  openvdb::Coord m_search_goal;
};
} // namespace astar_vdb
