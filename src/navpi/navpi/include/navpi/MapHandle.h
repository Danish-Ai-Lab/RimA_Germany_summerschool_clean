#ifndef PLANNER_EVAL_MAP_HANDLE_H_INCLUDED
#define PLANNER_EVAL_MAP_HANDLE_H_INCLUDED

#include <vdb_mapping/OccupancyVDBMapping.hpp>
#include <vdb_mapping/VDBMapping.hpp>

class MapHandle
{
  using ValueType  = float;
  using ValueGridT = openvdb::Grid<typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type>;

public:
  MapHandle(ValueGridT::Ptr grid,
            int dilate_level,
            int closing_level,
            int ground_search_level,
            double max_slope_angle,
            int step_height);
  ~MapHandle(){};


  bool generateValidNode(openvdb::Coord& coord, int max_lookup = 100);

  ValueGridT::Ptr getCspaceGrid() { return m_cspace_grid; }
  ValueGridT::Ptr getValueGrid() { return m_value_grid; }

  bool hasGround(openvdb::Coord coord, bool use_cache = true);
  bool isUnexplored(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc);
  bool isFree(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc);
  bool isOccupied(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc);
  bool isNodeValid(const openvdb::Coord coord);

  openvdb::Coord findClosestMapIndex(const openvdb::Coord& coord);
  void printSearchParameters();


private:
  // Ugly copy paste bullshit
  ValueGridT::Ptr m_value_grid;
  ValueGridT::Ptr m_cspace_grid;
  void inflateMap();
  void preprocessMap(typename ValueGridT::Ptr value_grid,
                     typename ValueGridT::Ptr& processed_grid,
                     typename ValueGridT::Ptr& cspace_grid);
  bool isSurfacePoint(openvdb::Coord coord, const std::shared_ptr<ValueGridT::Accessor> value_acc);
  bool isSlope(openvdb::Coord coord,
               const std::shared_ptr<ValueGridT::Accessor> value_acc,
               bool use_cache = true);

  void setMaxSlope(double angle);

  int m_dilate_level;         //  = 5;
  int m_closing_level;        //  = 3;
  int m_ground_search_level;  //= 12;
  int m_max_slope_cell_count; // = 11;
  int m_step_height;          // = 4;
  int m_slope_box_range;      // = 3;
  double m_max_slope_angle;
};


#endif // !PLANNER_EVAL_MAP_HANDLE_H_INCLUDED
