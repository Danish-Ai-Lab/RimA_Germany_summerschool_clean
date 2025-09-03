#include "planner_vdb_base/planner_vdb_base.h"
#include <assert.h>
#include <chrono>
#include <openvdb/tools/Morphology.h>


namespace planner_vdb_base {

void PlannerVDBBase::initPlanner(typename ValueGridT::Ptr value_grid)
{
  m_groundcache_grid = openvdb::BoolGrid::create();
  m_groundcache_grid->setTransform(openvdb::math::Transform::createLinearTransform(1));
  m_groundcache_grid->setGridClass(openvdb::GRID_LEVEL_SET);
  m_groundcache_grid_acc =
    std::make_shared<openvdb::BoolGrid::Accessor>(m_groundcache_grid->getAccessor());

  preprocessMap(value_grid, m_value_grid, m_cspace_grid);
  m_value_grid_acc  = std::make_shared<ValueGridT::Accessor>(m_value_grid->getAccessor());
  m_cspace_grid_acc = std::make_shared<ValueGridT::Accessor>(m_cspace_grid->getAccessor());

  if (m_debug_mode)
  {
    m_debug_grid = openvdb::FloatGrid::create();
    m_debug_grid->setTransform(value_grid->transformPtr());
    m_debug_grid->setGridClass(openvdb::GRID_LEVEL_SET);
  }

  m_ground_dist_grid = ValueGridT::create();
  m_ground_dist_grid->setTransform(value_grid->transformPtr());
  m_ground_dist_grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

void PlannerVDBBase::printSearchParameters()
{
  std::cout << std::endl;
  std::cout << "Search parameters:" << std::endl;
  std::cout << "------------------" << std::endl;
  std::cout << "m_debug_mode: " << m_debug_mode << std::endl;
  std::cout << "m_ground_search_level: " << m_ground_search_level << std::endl;
  std::cout << "m_ground_cache_on: " << m_ground_cache_on << std::endl;
  std::cout << "m_successor_variant: " << m_successor_variant << std::endl;
  std::cout << "m_successor_restriction: " << m_successor_restriction << std::endl;
  std::cout << "m_gravity_factor: " << m_gravity_factor << std::endl;
  std::cout << "m_floating_penalty: " << m_floating_penalty << std::endl;
  std::cout << "closing_level: " << closing_level << std::endl;
  std::cout << "dilate_level: " << dilate_level << std::endl;
  std::cout << "step_height: " << step_height << std::endl;
  std::cout << "slope_box_range: " << slope_box_range << std::endl;
  std::cout << "max_slope_angle: " << max_slope_angle << std::endl;
  std::cout << "max_slope cells: " << max_slope_cell_count << std::endl;
}

void PlannerVDBBase::cancelPlanning()
{
  std::cout << "Cancel of path planning requested." << std::endl;
  m_cancel = true;
};

std::vector<openvdb::Coord> PlannerVDBBase::getSuccessorCoords(const openvdb::Coord& coord)
{
  if (m_debug_mode)
  {
    get_succ_count++;
  }

  std::vector<openvdb::Coord> candidates;

  // 3D - 6-connected
  // Generates successors in 6 neighboring grid cells around the current cell
  // orthogonal neighbors
  if (m_successor_variant >= 0)
  {
    candidates.push_back(coord + openvdb::Coord(0, 0, -1));
    candidates.push_back(coord + openvdb::Coord(0, 0, 1));
    candidates.push_back(coord + openvdb::Coord(-1, 0, 0));
    candidates.push_back(coord + openvdb::Coord(0, -1, 0));
    candidates.push_back(coord + openvdb::Coord(0, 1, 0));
    candidates.push_back(coord + openvdb::Coord(1, 0, 0));
  }

  // 3D - 10-connected
  // Generates successors in 10 neighboring grid cells around the current cell
  // In plane: orthogonal and diagonal neighbors
  // In z-direction: orthogonal neighbors
  if (m_successor_variant >= 1)
  {
    candidates.push_back(coord + openvdb::Coord(-1, -1, 0));
    candidates.push_back(coord + openvdb::Coord(-1, 1, 0));
    candidates.push_back(coord + openvdb::Coord(1, -1, 0));
    candidates.push_back(coord + openvdb::Coord(1, 1, 0));
  }

  // 3D - 18-connected
  // Generates successors in 18 neighboring grid cells around the current cell
  // In z = 0 plane: orthogonal and diagonal neighbors
  // In z = 1, -1 plane: orthogonal neighbors
  if (m_successor_variant >= 2)
  {
    candidates.push_back(coord + openvdb::Coord(1, 0, -1));
    candidates.push_back(coord + openvdb::Coord(-1, 0, -1));
    candidates.push_back(coord + openvdb::Coord(0, 1, -1));
    candidates.push_back(coord + openvdb::Coord(0, -1, -1));
    candidates.push_back(coord + openvdb::Coord(1, 0, 1));
    candidates.push_back(coord + openvdb::Coord(-1, 0, 1));
    candidates.push_back(coord + openvdb::Coord(0, 1, 1));
    candidates.push_back(coord + openvdb::Coord(0, -1, 1));
  }

  // 3D - 26-connected
  // Generates successors in 26 neighboring grid cells around the current cell
  // All orthogonal and diagonal neighbors
  if (m_successor_variant >= 3)
  {
    candidates.push_back(coord + openvdb::Coord(-1, -1, -1));
    candidates.push_back(coord + openvdb::Coord(-1, 1, -1));
    candidates.push_back(coord + openvdb::Coord(1, -1, -1));
    candidates.push_back(coord + openvdb::Coord(1, 1, -1));
    candidates.push_back(coord + openvdb::Coord(-1, -1, 1));
    candidates.push_back(coord + openvdb::Coord(-1, 1, 1));
    candidates.push_back(coord + openvdb::Coord(1, -1, 1));
    candidates.push_back(coord + openvdb::Coord(1, 1, 1));
  }


  std::vector<openvdb::Coord> neighbors;


  for (auto n : candidates)
  {
    // No restriction
    if (m_successor_restriction == 0)
    {
      neighbors.push_back(n);
    }
    // can't restrict neighbors to much because of replanning functionality,
    // needs g-values of occupied neighbors if obstacle vanishes before replanning

    // not sure if this restriction is ok
    // but it is speeeed
    // object territory but not border of object which is important to take
    else if (m_successor_restriction == 1)
    {
      if (m_cspace_grid_acc->isValueOn(coord) && m_cspace_grid_acc->isValueOn(n))
      {
        continue;
      }

      if (isUnexplored(n, m_cspace_grid_acc) || hasGround(n, m_value_grid_acc))
      {
        neighbors.push_back(n);
      }
    }
    else if (m_successor_restriction == 2)
    {
      // if unknown terrain in cspace -> take it
      // if cspace occupied -> don't take it
      // if cspace free and ground under us -> take it
      if (isUnexplored(n, m_cspace_grid_acc) && hasGround(n, m_value_grid_acc)) // ok for
      // replanning
      {
        neighbors.push_back(n); // I belieeve I can flyyy
      }
      else if (isFree(n, m_cspace_grid_acc)
               // !m_cspace_grid_acc->isValueOn(n)
               && hasGround(n, m_value_grid_acc)) // ok for replanning
      {
        neighbors.push_back(n);
      }
    }
    else if (m_successor_restriction == 3)
    // could be good for replanning setup time if edges are not
    // excluded, ground is handled in get cost, performance?
    {
      if (isUnexplored(n, m_cspace_grid_acc) || isFree(n, m_cspace_grid_acc))
      {
        neighbors.push_back(n);
      }
    }
  }
  return neighbors;
}

double PlannerVDBBase::getHeuristic(const openvdb::Coord& current,
                                    const openvdb::Coord& start,
                                    const openvdb::Coord& goal)
{
  if (m_debug_mode)
  {
    get_heuristic_count++;
  }
  double heuristic    = 0;
  openvdb::Coord diff = current - goal;

  // For 6 directions of movement (Is the fastest in planes)
  // Manhatten Heuristik heuristik
  if (m_successor_variant == 0)
  {
    heuristic =
      (double)std::abs(diff.x()) + (double)std::abs(diff.y()) + (double)std::abs(diff.z());
  }

  // For 10 and 18 directions of movement
  // Schnell Heuristic
  if (m_successor_variant == 1 || m_successor_variant == 2)
  {
    std::pair<double, double> p =
      std::minmax<double>((double)std::abs(diff.x()), (double)std::abs(diff.y()));
    heuristic = (std::sqrt(2.0) * p.first + (p.second - p.first) + (double)std::abs(diff.z()));
  }

  // For 26 directions of movement
  // Mauch Heuristic ;-)
  if (m_successor_variant == 3)
  {
    double dx   = std::abs(diff.x());
    double dy   = std::abs(diff.y());
    double dz   = std::abs(diff.z());
    double dmin = std::min({dx, dy, dz});
    double dmax = std::max({dx, dy, dz});
    double dmid = dx + dy + dz - dmin - dmax;

    heuristic = dmax + (sqrt(2.0) - 1.0) * dmid + (sqrt(3.0) - sqrt(2.0)) * dmin;
  }

  // Mauch Tie breaker
  openvdb::Vec3d v_diff        = diff.asVec3d();
  openvdb::Vec3d v_coverd      = (start - current).asVec3d();
  openvdb::Vec3d v_total       = (start - goal).asVec3d();
  openvdb::Vec3d cross_product = v_diff.cross(v_total);
  double area_between          = cross_product.length();

  // Higher push factor means I will stay stronger along the line of sight
  // Measured that values between 0.01 and 0.1 work best do not go above 1.0
  double direction_push_factor = 0.1;
  heuristic += area_between / (1 + v_coverd.length()) * direction_push_factor;
  return heuristic;
}

double PlannerVDBBase::getCost(const openvdb::Coord& from,
                               const openvdb::Coord& to,
                               const std::shared_ptr<ValueGridT::Accessor> value_acc,
                               const std::shared_ptr<ValueGridT::Accessor> cspace_acc,
                               bool use_ground_cache)
{
  if (m_debug_mode)
  {
    get_cost_count++;
  }
  return getEuclideanDistance(from, to);


  // @Mauch: Generally restrictions should not be done here but instead in getSuccessorCoords()
  // @Mauch: This is restriction is also already coverd through m_successor_restriction=2
  // if (isOccupied(from, cspace_acc) || isOccupied(to, cspace_acc) ||
  //     // cspace_acc->isValueOn(from) || cspace_acc->isValueOn(to) ||
  //     !hasGround(from, value_acc, use_ground_cache) || !hasGround(to, value_acc,
  //     use_ground_cache))
  // {
  //   return std::numeric_limits<double>::infinity();
  // }


  // gravity: favor going down
  // double gravity_factor = 1;
  // // 1 if delta z = 0
  // // 0.9 if delta z < 0, going down
  // // 1.1 if delta z > 0, going up
  // int height_diff = (to - from).z();
  // if (height_diff < 0)
  // {
  //   gravity_factor = m_gravity_factor; // prefer going down if there would be a tie
  //   return 0;
  // }
  // else if (height_diff > 0)
  // {
  //   gravity_factor = m_floating_penalty;
  // }


  // if (!value_acc->isValueOn(to - openvdb::Coord(0, 0, dilate_level + 1)) ||
  //     !cspace_acc->isValueOn(to - openvdb::Coord(0, 0, 1)))
  // where is ground
  // int floating_distance = 2 * dilate_level;
  // float penalty_factor = 1.2;
  // if (m_ground_dist_grid->getAccessor().isValueOn(to))
  // {
  //   penalty_factor = 1 + m_ground_dist_grid->getAccessor().getValue(to) * 0.01;
  // }
  // else
  // {
  //   for (int z = 0; z <= step_height; ++z)
  //   {
  //     if (isSurfacePoint(to - openvdb::Coord(0, 0, dilate_level + z), value_acc))
  //     {
  //       penalty_factor = 1 + z * 0.01;
  //       // m_ground_dist_grid->getAccessor().setValueOn(to, z);
  //       break;
  //     }
  //   }
  // }


  // if (isFree(to - openvdb::Coord(0, 0, dilate_level + 1), value_acc) ||
  //     isFree(to - openvdb::Coord(0, 0, 1), cspace_acc))

  // {
  // penalty for floating
  // return m_floating_penalty * gravity_factor * getEuclideanDistance(from, to);
  //   return penalty_factor * getEuclideanDistance(from, to);
  // }

  // double penalty = 1.0;
  // if (isUnexplored(to, cspace_acc))
  // {
  //   penalty = std::sqrt(3);
  // }

  // return gravity_factor *
  //        getEuclideanDistance(from, to); // TODO use Schnell here as well? euclid is so expensive
}

void PlannerVDBBase::inflateMap(typename ValueGridT::Ptr input, typename ValueGridT::Ptr& output)
{
  m_cspace_grid = m_value_grid->deepCopy();
  openvdb::tools::dilateActiveValues(m_cspace_grid->tree(),
                                     dilate_level,
                                     openvdb::tools::NN_FACE_EDGE_VERTEX,
                                     openvdb::tools::EXPAND_TILES,
                                     true);
}

void PlannerVDBBase::preprocessMap(typename ValueGridT::Ptr value_grid,
                                   typename ValueGridT::Ptr& processed_grid,
                                   typename ValueGridT::Ptr& cspace_grid)
{
  auto t1_preprocess = std::chrono::high_resolution_clock::now();
  // close holes in map
  processed_grid = value_grid->deepCopy();

  auto t1_closing = std::chrono::high_resolution_clock::now();
  openvdb::tools::dilateActiveValues(processed_grid->tree(),
                                     closing_level,
                                     openvdb::tools::NN_FACE_EDGE_VERTEX,
                                     openvdb::tools::EXPAND_TILES,
                                     true);
  openvdb::tools::erodeActiveValues(
    processed_grid->tree(),
    closing_level,
    openvdb::tools::NN_FACE_EDGE_VERTEX,
    openvdb::tools::EXPAND_TILES,
    true); // https://github.com/AcademySoftwareFoundation/openvdb/issues/1397
  auto t2_closing = std::chrono::high_resolution_clock::now();

  auto t1_inflate = std::chrono::high_resolution_clock::now();
  // c space map
  inflateMap(processed_grid, cspace_grid);
  auto t2_inflate = std::chrono::high_resolution_clock::now();

  auto t2_preprocess = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_preprocess = t2_preprocess - t1_preprocess;
  std::cout << std::endl;
  std::cout << "Planner map preprocessing:" << std::endl;
  std::cout << "--------------------------" << std::endl;
  if (m_debug_mode)
  {
    std::chrono::duration<double, std::milli> ms_closing = t2_closing - t1_closing;
    std::chrono::duration<double, std::milli> ms_inflate = t2_inflate - t1_inflate;
    std::cout << "Time for map closing: " << ms_closing.count() << "ms\n";
    std::cout << "Time for map inflation: " << ms_inflate.count() << "ms\n";
  }
  std::cout << "Time total: " << ms_preprocess.count() << "ms\n";
}

bool PlannerVDBBase::isSurfacePoint(openvdb::Coord coord,
                                    const std::shared_ptr<ValueGridT::Accessor> value_acc)
{
  // return value_acc->isValueOn(coord) && !value_acc->isValueOn(coord + openvdb::Coord(0, 0, 1));
  return isOccupied(coord, value_acc) && isFree(coord + openvdb::Coord(0, 0, 1), value_acc);
}


bool PlannerVDBBase::isSlope(openvdb::Coord coord,
                             const std::shared_ptr<ValueGridT::Accessor> value_acc,
                             bool use_cache)
{
  static bool once = false;
  // get corner points
  // m_debug_grid->clear();
  // if (!once)
  // {
  //   std::cout << "////////////////////////////////////////////// " << std::endl;
  //   std::cout << "coord " << coord << std::endl;
  // }


  // ground should be at coord - dilation
  auto ground_level = coord - openvdb::Coord(0, 0, dilate_level + 1);

  // outer
  std::vector<openvdb::Coord> test_positions;
  openvdb::Coord test_pos;


  test_positions.push_back(ground_level + openvdb::Coord(-dilate_level, dilate_level, 0));  // A
  test_positions.push_back(ground_level + openvdb::Coord(dilate_level, dilate_level, 0));   // B
  test_positions.push_back(ground_level + openvdb::Coord(-dilate_level, -dilate_level, 0)); // C
  test_positions.push_back(ground_level + openvdb::Coord(dilate_level, -dilate_level, 0));  // D

  // find corner points
  std::vector<openvdb::Coord> corner_points;


  for (size_t i = 0; i < test_positions.size(); ++i)
  {
    for (int z = 10; z >= -10; --z)
    {
      test_pos = test_positions[i] + openvdb::Coord(0, 0, z);
      if (isSurfacePoint(test_pos, value_acc))
      {
        // if (!once)
        // {
        // if (m_debug_mode){
        //   m_debug_grid->getAccessor().setValueOn(test_pos, 1);
        // }
        // }

        corner_points.push_back(test_pos);
        break;
      }
    }
  }
  // once = true;

  if (corner_points.size() < 4)
  {
    return false;
  }

  auto a_c = corner_points[0] - corner_points[2];
  auto d_c = corner_points[3] - corner_points[2];

  // check the slope
  if (std::abs(a_c.z()) > max_slope_cell_count || std::abs(d_c.z()) > max_slope_cell_count)
  {
    return false;
  }


  // check if corner points enclose test position
  // prevents exploring of points higher than step height on a plane
  // int min_z = corner_points[0].z();
  int max_z = corner_points[0].z();
  for (size_t i = 1; i < corner_points.size(); ++i)
  {
    // min_z = std::min(min_z, corner_points[i].z());
    max_z = std::max(max_z, corner_points[i].z());
  }
  // min_z -= step_height;
  // max_z += step_height;

  if (ground_level.z() > max_z + 1) //|| ground_level.z() < min_z)
  {
    // std::cout << "coord: " << coord << std::endl;
    // std::cout << "min_z: " << min_z << std::endl;
    // std::cout << "max_z: " << max_z << std::endl;
    // std::cout << "ground_z: " << ground_level.z() << std::endl;
    // assert(false);
    return false;
  }

  bool surface_found = false;

  // if (!once)
  // {
  //   std::cout << "A " << corner_points[0] << std::endl;
  //   std::cout << "B " << corner_points[1] << std::endl;
  //   std::cout << "C " << corner_points[2] << std::endl;
  //   std::cout << "D " << corner_points[3] << std::endl;
  // }

  int step = 1;
  if (m_ground_search_level == 11)
  {
    step = dilate_level;
  }
  else if (m_ground_search_level == 12)
  {
    step = dilate_level / 2;
  }


  // for (int y = 0; y <= 2 * dilate_level + 1; y += dilate_level)
  for (int y = 0; y <= 2 * dilate_level + 1; y += step)
  {
    // test_y = C + y / (2 * dilate_level + 1) * (A - C);
    float y_multiplier = (float)y / (2.0 * dilate_level + 1);

    // for (int x = 0; x <= 2 * dilate_level + 1; x += dilate_level)
    for (int x = 0; x <= 2 * dilate_level + 1; x += step)
    {
      // test_x = C + x / (2 * dilate_level + 1) * (D - C);
      float x_multiplier = (float)x / (2.0 * dilate_level + 1);

      surface_found = false;

      openvdb::Coord test_pos = corner_points[2] +
                                openvdb::Coord((int)(x_multiplier * (float)d_c.x()),
                                               (int)(x_multiplier * (float)d_c.y()),
                                               (int)(x_multiplier * (float)(d_c.z()))) +
                                openvdb::Coord((int)(y_multiplier * (float)a_c.x()),
                                               (int)(y_multiplier * (float)a_c.y()),
                                               (int)(y_multiplier * (float)a_c.z()));

      for (int z = slope_box_range; z >= -slope_box_range; --z)
      {
        if (!once)
        {
          if (m_debug_mode)
          {
            m_debug_grid->getAccessor().setValueOn(test_pos + openvdb::Coord(0, 0, z), 1);
          }
          // std::cout << "xmult " << x_multiplier << std::endl;
          // std::cout << "ymult " << y_multiplier << std::endl;
          // std::cout << test_pos + openvdb::Coord(0, 0, z) << std::endl;
        }
        if (isSurfacePoint(test_pos + openvdb::Coord(0, 0, z), value_acc) ||
            isUnexplored(test_pos + openvdb::Coord(0, 0, z), value_acc))
        {
          // std::cout << "found " << test_pos << std::endl;

          surface_found = true;
          break;
        }
      }
      if (!surface_found)
      {
        // std::cout << "nope" << std::endl;
        // assert(false);
        once = true;
        return false;
      }
    }
  }
  // std::cout << "yep" << std::endl;
  // assert(false);
  once = true;

  return true;
}

bool PlannerVDBBase::hasGround(openvdb::Coord coord,
                               const std::shared_ptr<ValueGridT::Accessor> value_acc,
                               bool use_cache)
{
  if (m_debug_mode)
  {
    has_ground_count++;
  }
  // lookup in cache
  m_groundcache_grid_acc =
    std::make_shared<openvdb::BoolGrid::Accessor>(m_groundcache_grid->getAccessor());
  if (m_ground_cache_on && use_cache && m_groundcache_grid_acc->isValueOn(coord))
  {
    if (m_debug_mode)
    {
      chache_hit_count++;
    }
    return m_groundcache_grid_acc->getValue(coord);
  }

  // ground should be at coord - dilation
  auto ground_level = coord - openvdb::Coord(0, 0, dilate_level + 1);

  /////////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<openvdb::Coord> test_positions;
  openvdb::Coord test_pos;
  if (m_ground_search_level == 0)
  {
    for (int y = -1; y <= 1; ++y)
    {
      for (int x = -1; x <= 1; ++x)
      {
        test_positions.push_back(ground_level + openvdb::Coord(x, y, 0));
      }
    }
  }
  else if (m_ground_search_level == 3 || m_ground_search_level == 13)
  {
    for (int y = -dilate_level; y <= dilate_level; ++y)
    {
      for (int x = -dilate_level; x <= dilate_level; ++x)
      {
        test_positions.push_back(ground_level + openvdb::Coord(x, y, 0));
      }
    }
  }
  else // level 1 and 2
  {
    // outer
    // level 1, 11, 2, 12
    test_positions.push_back(ground_level + openvdb::Coord(dilate_level, dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(-dilate_level, dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(-dilate_level, -dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(dilate_level, -dilate_level, 0));

    if (m_ground_search_level == 2 || m_ground_search_level == 12)
    {
      // quadrant centers
      test_positions.push_back(ground_level +
                               openvdb::Coord(-dilate_level / 2, -dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(-dilate_level / 2, dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(dilate_level / 2, dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(dilate_level / 2, -dilate_level / 2, 0));

      // remaining quadrant corners
      test_positions.push_back(ground_level);
      test_positions.push_back(ground_level + openvdb::Coord(0, -dilate_level, 0));
      test_positions.push_back(ground_level + openvdb::Coord(-dilate_level, 0, 0));
      test_positions.push_back(ground_level + openvdb::Coord(0, dilate_level, 0));
      test_positions.push_back(ground_level + openvdb::Coord(dilate_level, 0, 0));
    }
  }


  bool has_ground    = true;
  bool surface_found = false;

  // make sure that every test position has a surface point in the range of (-stepheight,0)
  for (size_t i = 0; i < test_positions.size(); ++i)
  {
    surface_found = false;
    for (int z = 0; z >= -step_height; --z)
    {
      test_pos = test_positions[i] + openvdb::Coord(0, 0, z);
      if (isSurfacePoint(test_pos, value_acc) ||
          isUnexplored(test_pos, value_acc) // allow exploration of new area
          // if we want to be safer to not fall from bridges etc we need to use m_cspace_grid_acc
          // instead of m_value_grid_acc but then we can't overcome unexplored gaps
      )
      {
        surface_found = true;
        break;
      }
    }
    if (!surface_found)
    {
      has_ground = false;
      break;
    }
  }

  if (m_ground_search_level > 10 && !has_ground)
  {
    // check slope
    has_ground = isSlope(coord, value_acc, use_cache);
  }

  if (m_ground_cache_on && use_cache)
  {
    m_groundcache_grid_acc->setValueOn(coord, has_ground);
  }
  return has_ground;
}

bool PlannerVDBBase::isUnexplored(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
{
  return
    // acc->getValue(coord) < -0.1 ||
    !acc->isValueOn(coord) && acc->getValue(coord) == 0;
}

bool PlannerVDBBase::isNodeValid(openvdb::Coord coord, const typename ValueGridT::Ptr grid)
{
  auto acc = grid->getAccessor();

  // valid node if
  // unexplored and has ground
  // cspace free and has ground
  if ((isUnexplored(coord, m_cspace_grid_acc) || !m_cspace_grid_acc->isValueOn(coord)) &&
      hasGround(coord, m_value_grid_acc))
  // if (!m_cspace_grid_acc->isValueOn(coord) && hasGround(coord, m_value_grid_acc))
  // if(hasGround(coord, m_value_grid_acc ))
  {
    return true;
  }
  return false;

  // Checks if the current coordinate is free and the neighbor below it is occupied
  // return !acc.isValueOn(coord) && acc.isValueOn(coord + openvdb::Coord(0, 0, -1));
}

bool PlannerVDBBase::generateValidNode(openvdb::Coord& coord, const typename ValueGridT::Ptr grid)
{
  auto acc = grid->getAccessor();

  bool found     = false;
  int i          = 0;
  int max_lookup = 100;

  std::cout << "Trying to find valid node for: " << coord << std::endl;

  std::vector<openvdb::Coord> unknown_ground;

  while (!found)
  {
    if (i > max_lookup)
    {
      break;
    }
    openvdb::Coord modifier(0, 0, i);
    if (isNodeValid(coord + modifier, grid))
    {
      if (!isUnexplored(coord + modifier, m_value_grid_acc))
      {
        coord += modifier;
        std::cout << "Found valid node above at " << coord << std::endl;
        return true;
      }
      else
      {
        unknown_ground.push_back(coord + modifier);
      }
    }
    if (isNodeValid(coord - modifier, grid))
    {
      if (!isUnexplored(coord - modifier, m_value_grid_acc))
      {
        coord -= modifier;
        std::cout << "Found valid node below at " << coord << std::endl;
        return true;
      }
      else
      {
        unknown_ground.push_back(coord + modifier);
      }
    }
    i++;
  }
  std::cout << "Not able to find true ground point. Fallback to unknown ground." << std::endl;
  if (unknown_ground.empty())
  {
    std::cout << "No valid unknown ground found. Trying with input pose." << std::endl;
    return false;
  }
  coord = unknown_ground[0];
  std::cout << "Valid Point at " << coord << std::endl;
  return true;
}


double PlannerVDBBase::getEuclideanDistance(const openvdb::Coord& from, const openvdb::Coord& to)
{
  auto diff = from - to;
  auto l1   = (double)diff.asVec3d().length();
  // auto l2   = (double)std::sqrt(std::pow((double)diff.x(), 2) + std::pow((double)diff.y(), 2) +
  //                            std::pow((double)diff.z(), 2));
  // assert(doubleEquals(l1, l2));
  return l1;
}

SearchState PlannerVDBBase::computeShortestPath()
{
  auto t1 = std::chrono::high_resolution_clock::now();

  do
  {
    step();
  } while (m_state == planner_vdb_base::SearchState::SEARCHING);

  auto t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> ms = t2 - t1;
  std::cout << "compute shortest path done in " << ms.count() << "ms" << std::endl;

  printSearchStats();

  return m_state;
}


void PlannerVDBBase::setMaxSlopeAngle(float alpha)
{
  max_slope_angle      = alpha;
  max_slope_cell_count = tan(degreeToRadian(max_slope_angle)) * (2.0 * dilate_level + 1);
}

} // namespace planner_vdb_base
