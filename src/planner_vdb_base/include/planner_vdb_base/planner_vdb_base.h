#pragma once

#include <memory>
#include <openvdb/openvdb.h>
#include <vector>

#include "planner_utils.h"


namespace planner_vdb_base {
enum class SearchState
{
  NOT_INITIALIZED,
  READY,
  SEARCHING,
  SUCCESS,
  FAILURE
};

using ValueType  = float;
using ValueGridT = openvdb::Grid<typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type>;


class PlannerVDBBase
{
public:
  PlannerVDBBase(bool debug_mode               = false,
                 int ground_seach_level        = 1,
                 bool ground_cache_on          = true,
                 int successor_variant         = 3,
                 int successor_restriction     = 2,
                 int stepheight                = 4,
                 int dilate                    = 5,
                 int slope_ground_range        = 2,
                 float max_slope_angle_degrees = 45)
    : m_debug_mode(debug_mode)
    , m_ground_search_level(ground_seach_level)
    , m_ground_cache_on(ground_cache_on)
    , m_successor_variant(successor_variant)
    , m_successor_restriction(successor_restriction)
    , step_height(stepheight)
    , dilate_level(dilate)
  // , closing_level(dilate)
  {
    setMaxSlopeAngle(max_slope_angle_degrees);
    slope_box_range = slope_ground_range;

    printSearchParameters();
  }

  /**
   * Find a path from start to goal.
   * Start and Goal must be set before calling this with setStartAndGoal
   *
   * \return state in which the search ended
   **/
  virtual SearchState computeShortestPath();


  /**
   * Perform one step of the path finding algorithm
   * Start and Goal must be set before calling thist with setStartAndGoal
   * \return state in which the search step ended
   **/
  virtual SearchState step() = 0;

  /**
   * Set the start and goal for the path planning
   *
   * \param start the start coordinate
   * \param goal the goal coordinate
   **/
  virtual bool setStartAndGoal(const openvdb::Coord& start, const openvdb::Coord& goal) = 0;

  /**
   * Extract the found path
   *
   * \return vector of path coordinates
   **/
  virtual std::vector<openvdb::Coord> extractPath() = 0;

  /**
   * Requests to cancel the path planning
   * */
  virtual void cancelPlanning();

  virtual void printSearchStats() = 0;
  virtual void printSearchParameters();

  virtual bool isPathAvailable() = 0;

  openvdb::Coord getStart() { return m_start_index_coord; };
  openvdb::Coord getGoal() { return m_goal_index_coord; };

  typename ValueGridT::Ptr getCSpaceGrid() { return m_cspace_grid; };
  typename ValueGridT::Ptr getProcessedGrid() { return m_value_grid; };
  typename ValueGridT::Ptr getDebugGrid() { return m_debug_grid; };


protected:
  SearchState m_state = SearchState::NOT_INITIALIZED;

  bool m_cancel           = false;
  const bool m_debug_mode = false;

  // 0 = A* ground
  // 1 = base
  // 2 = intermediate
  // 3 = full
  const int m_ground_search_level = 3;
  const bool m_ground_cache_on    = true;

  // 0 = 6 directions of movement
  // 1 = 10 directions of movement
  // 2 = 18 directions of movement
  // 3 = 26 directions of movement
  const int m_successor_variant = 3;

  // 0 = No restriction
  // 1 = Todo: Explain
  // 2 = Todo: Explain
  // 3 = Todo: Explain
  const int m_successor_restriction = 2;
  const double m_gravity_factor     = 0.9;
  const double m_floating_penalty   = 1.1;

  long chache_hit_count    = 0;
  long has_ground_count    = 0;
  long get_heuristic_count = 0;
  long get_cost_count      = 0;
  long get_succ_count      = 0;
  long node_expand_count   = 0;


  const int closing_level = 3;
  int step_height         = 4;
  int dilate_level        = 5; // should represent half of robot dimension
  //~45 degrees, in cells, max_slope = tan(alpha) * (2*dilate_level+1)
  int max_slope_cell_count = 11;
  float max_slope_angle    = 45;
  int slope_box_range      = step_height / 2;

  openvdb::Coord m_start_index_coord;
  openvdb::Coord m_goal_index_coord;


  typename ValueGridT::Ptr m_value_grid; // occupancy value
  typename ValueGridT::Ptr m_cspace_grid;
  typename ValueGridT::Ptr m_ground_dist_grid;
  openvdb::BoolGrid::Ptr m_groundcache_grid;
  typename ValueGridT::Ptr m_debug_grid;

  // TODO: dont store accessors as member
  std::shared_ptr<ValueGridT::Accessor> m_value_grid_acc;  // holds map values
  std::shared_ptr<ValueGridT::Accessor> m_cspace_grid_acc; // holds map values
  std::shared_ptr<openvdb::BoolGrid::Accessor> m_groundcache_grid_acc;

  virtual void initPlanner(typename ValueGridT::Ptr value_grid);

  virtual std::vector<openvdb::Coord> getSuccessorCoords(const openvdb::Coord& coord);

  virtual double getHeuristic(const openvdb::Coord& current,
                              const openvdb::Coord& start,
                              const openvdb::Coord& goal);

  virtual double getCost(const openvdb::Coord& from,
                         const openvdb::Coord& to,
                         const std::shared_ptr<ValueGridT::Accessor> value_acc,
                         const std::shared_ptr<ValueGridT::Accessor> cspace_acc,
                         bool use_ground_cache = true);

  virtual void preprocessMap(typename ValueGridT::Ptr value_grid,
                             typename ValueGridT::Ptr& processed_grid,
                             typename ValueGridT::Ptr& cspace_grid);

  virtual void inflateMap(typename ValueGridT::Ptr input, typename ValueGridT::Ptr& output);

  virtual bool hasGround(openvdb::Coord coord,
                         const std::shared_ptr<ValueGridT::Accessor> value_acc,
                         bool use_cache = true);

  virtual bool isSlope(openvdb::Coord coord,
                       const std::shared_ptr<ValueGridT::Accessor> value_acc,
                       bool use_cache = true);

  virtual bool isUnexplored(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc);
  virtual bool isFree(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
  {
    return !acc->isValueOn(coord); // || acc->getValue(coord) < -0.1;
  }
  virtual bool isOccupied(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
  {
    return acc->isValueOn(coord); // || acc->getValue(coord) > -0.1;
  }

  virtual bool isNodeValid(openvdb::Coord coord, const typename ValueGridT::Ptr grid);
  virtual bool generateValidNode(openvdb::Coord& coord, const typename ValueGridT::Ptr grid);
  virtual bool isSurfacePoint(openvdb::Coord coord,
                              const std::shared_ptr<ValueGridT::Accessor> value_acc);

  double getEuclideanDistance(const openvdb::Coord& from, const openvdb::Coord& to);

  void setMaxSlopeAngle(float alpha);
};
} // namespace planner_vdb_base
