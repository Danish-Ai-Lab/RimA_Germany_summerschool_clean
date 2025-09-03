#include <navpi/MapHandle.h>
#include <vdb_mapping_ros2/VDBMappingTools.hpp>

MapHandle::MapHandle(ValueGridT::Ptr grid,
                     int dilate_level,
                     int closing_level,
                     int ground_search_level,
                     double max_slope_angle,
                     int step_height)
{
  std::cout << "Setting Parameters for Path Planning" << std::endl;
  m_dilate_level        = dilate_level;
  m_closing_level       = closing_level;
  m_ground_search_level = ground_search_level;
  m_step_height         = step_height;
  m_slope_box_range     = (int)((m_step_height + 0.5) / 2.0);
  setMaxSlope(max_slope_angle);
  printSearchParameters();
  std::cout << "preprocessing map" << std::endl;
  preprocessMap(grid, m_value_grid, m_cspace_grid);
  std::cout << "preprocessing map done" << std::endl;
}

// Copy paste bullshit
void MapHandle::inflateMap()
{
  m_cspace_grid = m_value_grid->deepCopy();
  openvdb::tools::dilateActiveValues(m_cspace_grid->tree(),
                                     m_dilate_level,
                                     openvdb::tools::NN_FACE_EDGE_VERTEX,
                                     openvdb::tools::EXPAND_TILES,
                                     true);
}
void MapHandle::preprocessMap(typename ValueGridT::Ptr input_grid,
                              typename ValueGridT::Ptr& smoothed_grid,
                              typename ValueGridT::Ptr& cspace_grid)
{
  auto t1_preprocess = std::chrono::high_resolution_clock::now();
  // close holes in map
  std::cout << "creating deep copy" << std::endl;
  smoothed_grid = input_grid->deepCopy();
  std::cout << "creating deep copy done " << std::endl;

  std::cout << "Filtering..." << std::endl;
  auto t1_opening = std::chrono::high_resolution_clock::now();

  auto acc                      = smoothed_grid->getAccessor();
  int free_floating_voxel_count = 0;
  for (auto iter = smoothed_grid->cbeginValueOn(); iter.test(); ++iter)
  {
    int neighbor_count = 0;
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(1, 0, 0));
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(-1, 0, 0));
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(0, 1, 0));
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(0, -1, 0));
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(0, 0, 1));
    neighbor_count += acc.isValueOn(iter.getCoord() + openvdb::Coord(0, 0, -1));
    if (neighbor_count <= 1)
    {
      acc.setActiveState(iter.getCoord(), false);
      free_floating_voxel_count++;
    }
  }
  std::cout << "Filtered " << free_floating_voxel_count << " free floating voxels from the map"
            << std::endl;


  std::cout << "Filtering done" << std::endl;
  auto t2_opening = std::chrono::high_resolution_clock::now();


  std::cout << "closing" << std::endl;
  auto t1_closing = std::chrono::high_resolution_clock::now();
  openvdb::tools::dilateActiveValues(smoothed_grid->tree(),
                                     m_closing_level,
                                     openvdb::tools::NN_FACE_EDGE_VERTEX,
                                     openvdb::tools::EXPAND_TILES,
                                     true);
  openvdb::tools::erodeActiveValues(
    smoothed_grid->tree(),
    m_closing_level,
    openvdb::tools::NN_FACE_EDGE_VERTEX,
    openvdb::tools::EXPAND_TILES,
    true); // https://github.com/AcademySoftwareFoundation/openvdb/issues/1397


  auto t2_closing = std::chrono::high_resolution_clock::now();
  std::cout << "closing done" << std::endl;

  std::cout << "inflate" << std::endl;
  auto t1_inflate = std::chrono::high_resolution_clock::now();
  // c space map
  inflateMap();
  auto t2_inflate = std::chrono::high_resolution_clock::now();
  std::cout << "inflate done" << std::endl;

  auto t2_preprocess = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_preprocess = t2_preprocess - t1_closing;
  std::cout << std::endl;
  std::cout << "Planner map preprocessing:" << std::endl;
  std::cout << "--------------------------" << std::endl;
  std::cout << "Time total: " << ms_preprocess.count() << "ms\n";
}

bool MapHandle::isSurfacePoint(openvdb::Coord coord,
                               const std::shared_ptr<ValueGridT::Accessor> value_acc)
{
  // return value_acc->isValueOn(coord) && !value_acc->isValueOn(coord + openvdb::Coord(0, 0, 1));
  return isOccupied(coord, value_acc) && isFree(coord + openvdb::Coord(0, 0, 1), value_acc);
}
bool MapHandle::isSlope(openvdb::Coord coord,
                        const std::shared_ptr<ValueGridT::Accessor> value_acc,
                        bool use_cache)
{
  // ground should be at coord - dilation
  auto ground_level = coord - openvdb::Coord(0, 0, m_dilate_level + 1);

  // outer
  std::vector<openvdb::Coord> test_positions;
  openvdb::Coord test_pos;


  test_positions.push_back(ground_level + openvdb::Coord(-m_dilate_level, m_dilate_level, 0));  // A
  test_positions.push_back(ground_level + openvdb::Coord(m_dilate_level, m_dilate_level, 0));   // B
  test_positions.push_back(ground_level + openvdb::Coord(-m_dilate_level, -m_dilate_level, 0)); // C
  test_positions.push_back(ground_level + openvdb::Coord(m_dilate_level, -m_dilate_level, 0));  // D

  // find corner points
  std::vector<openvdb::Coord> corner_points;


  for (int i = 0; i < test_positions.size(); ++i)
  {
    for (int z = 10; z >= -10; --z)
    {
      test_pos = test_positions[i] + openvdb::Coord(0, 0, z);
      if (isSurfacePoint(test_pos, value_acc))
      {
        corner_points.push_back(test_pos);
        break;
      }
    }
  }

  if (corner_points.size() < 4)
  {
    return false;
  }

  auto a_c = corner_points[0] - corner_points[2];
  auto d_c = corner_points[3] - corner_points[2];

  // check the slope
  if (std::abs(a_c.z()) > m_max_slope_cell_count || std::abs(d_c.z()) > m_max_slope_cell_count)
  {
    return false;
  }


  // check if corner points enclose test position
  // prevents exploring of points higher than step height on a plane
  // int min_z = corner_points[0].z();
  int max_z = corner_points[0].z();
  for (int i = 1; i < corner_points.size(); ++i)
  {
    // min_z = std::min(min_z, corner_points[i].z());
    max_z = std::max(max_z, corner_points[i].z());
  }
  // min_z -= m_step_height;
  // max_z += m_step_height;

  if (ground_level.z() > max_z + 1) //|| ground_level.z() < min_z)
  {
    return false;
  }

  bool surface_found = false;

  int step = 1;
  if (m_ground_search_level == 11)
  {
    step = m_dilate_level;
  }
  else if (m_ground_search_level == 12)
  {
    step = m_dilate_level / 2;
  }


  // for (int y = 0; y <= 2 * m_dilate_level + 1; y += m_dilate_level)
  for (int y = 0; y <= 2 * m_dilate_level + 1; y += step)
  {
    // test_y = C + y / (2 * m_dilate_level + 1) * (A - C);
    float y_multiplier = (float)y / (2.0 * m_dilate_level + 1);

    // for (int x = 0; x <= 2 * m_dilate_level + 1; x += m_dilate_level)
    for (int x = 0; x <= 2 * m_dilate_level + 1; x += step)
    {
      // test_x = C + x / (2 * m_dilate_level + 1) * (D - C);
      float x_multiplier = (float)x / (2.0 * m_dilate_level + 1);

      surface_found = false;

      openvdb::Coord test_pos = corner_points[2] +
                                openvdb::Coord((int)(x_multiplier * (float)d_c.x()),
                                               (int)(x_multiplier * (float)d_c.y()),
                                               (int)(x_multiplier * (float)(d_c.z()))) +
                                openvdb::Coord((int)(y_multiplier * (float)a_c.x()),
                                               (int)(y_multiplier * (float)a_c.y()),
                                               (int)(y_multiplier * (float)a_c.z()));

      for (int z = m_slope_box_range; z >= -m_slope_box_range; --z)
      {
        if (isSurfacePoint(test_pos + openvdb::Coord(0, 0, z), value_acc) ||
            isUnexplored(test_pos + openvdb::Coord(0, 0, z), value_acc))
        {
          surface_found = true;
          break;
        }
      }
      if (!surface_found)
      {
        return false;
      }
    }
  }

  return true;
}

bool MapHandle::hasGround(openvdb::Coord coord, bool use_cache)
{
  auto value_grid_acc = std::make_shared<ValueGridT::Accessor>(m_value_grid->getAccessor());

  // TODO this part is currently rather hacky and the real check should be reintegrated again
  for (int i = 0; i < 3 * m_dilate_level; i++)
  {
    if (isOccupied(coord + openvdb::Coord(0, 0, -i), value_grid_acc))
    {
      return true;
    }
  }
  return false;

  // ground should be at coord - dilation
  auto ground_level = coord - openvdb::Coord(0, 0, m_dilate_level + 1);

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
    for (int y = -m_dilate_level; y <= m_dilate_level; ++y)
    {
      for (int x = -m_dilate_level; x <= m_dilate_level; ++x)
      {
        test_positions.push_back(ground_level + openvdb::Coord(x, y, 0));
      }
    }
  }
  else // level 1 and 2
  {
    // outer
    // level 1, 11, 2, 12
    test_positions.push_back(ground_level + openvdb::Coord(m_dilate_level, m_dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(-m_dilate_level, m_dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(-m_dilate_level, -m_dilate_level, 0));
    test_positions.push_back(ground_level + openvdb::Coord(m_dilate_level, -m_dilate_level, 0));

    if (m_ground_search_level == 2 || m_ground_search_level == 12)
    {
      // quadrant centers
      test_positions.push_back(ground_level +
                               openvdb::Coord(-m_dilate_level / 2, -m_dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(-m_dilate_level / 2, m_dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(m_dilate_level / 2, m_dilate_level / 2, 0));
      test_positions.push_back(ground_level +
                               openvdb::Coord(m_dilate_level / 2, -m_dilate_level / 2, 0));

      // remaining quadrant corners
      test_positions.push_back(ground_level);
      test_positions.push_back(ground_level + openvdb::Coord(0, -m_dilate_level, 0));
      test_positions.push_back(ground_level + openvdb::Coord(-m_dilate_level, 0, 0));
      test_positions.push_back(ground_level + openvdb::Coord(0, m_dilate_level, 0));
      test_positions.push_back(ground_level + openvdb::Coord(m_dilate_level, 0, 0));
    }
  }


  bool has_ground    = true;
  bool surface_found = false;

  // make sure that every test position has a surface point in the range of (-stepheight,0)
  for (int i = 0; i < test_positions.size(); ++i)
  {
    surface_found = false;
    for (int z = 0; z >= -m_step_height; --z)
    {
      test_pos = test_positions[i] + openvdb::Coord(0, 0, z);
      if (isSurfacePoint(test_pos, value_grid_acc) ||
          isUnexplored(test_pos, value_grid_acc) // allow exploration of new area
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
    has_ground = isSlope(coord, value_grid_acc, use_cache);
  }

  // if (m_ground_cache_on && use_cache)
  //{
  // m_groundcache_grid_acc->setValueOn(coord, has_ground);
  //}
  return has_ground;
}

bool MapHandle::isUnexplored(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
{
  return false;
  return
    // acc->getValue(coord) < -0.1 ||
    !acc->isValueOn(coord) && acc->getValue(coord) == 0;
}

bool MapHandle::isFree(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
{
  return !acc->isValueOn(coord); // || acc->getValue(coord) < -0.1;
}
bool MapHandle::isOccupied(openvdb::Coord coord, std::shared_ptr<ValueGridT::Accessor> acc)
{
  return acc->isValueOn(coord); // || acc->getValue(coord) > -0.1;
}

bool MapHandle::isNodeValid(const openvdb::Coord coord)
{
  auto value_grid_acc   = std::make_shared<ValueGridT::Accessor>(m_value_grid->getAccessor());
  auto c_space_grid_acc = std::make_shared<ValueGridT::Accessor>(m_cspace_grid->getAccessor());

  // valid node if
  // unexplored and has ground
  // cspace free and has ground
  // if ((isUnexplored(coord, c_space_grid_acc) || !c_space_grid_acc->isValueOn(coord)) &&
  // hasGround(coord, value_grid_acc))
  // if (!m_cspace_grid_acc->isValueOn(coord) && hasGround(coord, m_value_grid_acc))
  // if(hasGround(coord, m_value_grid_acc ))
  if (!c_space_grid_acc->isValueOn(coord) && hasGround(coord))
  {
    return true;
  }
  return false;

  // Checks if the current coordinate is free and the neighbor below it is occupied
  // return !acc.isValueOn(coord) && acc.isValueOn(coord + openvdb::Coord(0, 0, -1));
}

bool MapHandle::generateValidNode(openvdb::Coord& coord, int max_lookup)
{
  bool found = false;
  int i      = 0;

  while (!found)
  {
    if (i > max_lookup)
    {
      break;
    }
    openvdb::Coord modifier(0, 0, i);
    if (isNodeValid(coord + modifier))
    {
      coord += modifier;
      return true;
    }
    if (isNodeValid(coord - modifier))
    {
      coord -= modifier;
      return true;
    }
    i++;
  }
  return false;
}

openvdb::Coord MapHandle::findClosestMapIndex(const openvdb::Coord& coord)
{
  // TODO do this only in the current root leaf node to increase performance
  openvdb::Coord closest_point;
  double closest_distance;

  closest_point    = m_cspace_grid->cbeginValueOn().getCoord();
  closest_distance = (coord - closest_point).asVec3d().length();
  for (auto iter = m_cspace_grid->cbeginValueOn(); iter.test(); ++iter)
  {
    double distance = (coord - iter.getCoord()).asVec3d().length();
    if (distance < closest_distance)
    {
      if (isNodeValid(iter.getCoord() + openvdb::Coord(0, 0, 1)))
      {
        closest_distance = distance;
        closest_point    = iter.getCoord();
      }
    }
  }
  std::cout << "Found closest: " << closest_point << std::endl;
  std::cout << "Distance: " << closest_distance << std::endl;
  return closest_point;
}
void MapHandle::setMaxSlope(double angle)
{
  m_max_slope_angle      = angle;
  m_max_slope_cell_count = std::tan((angle * M_PI) / 180.0) * (2.0 * m_dilate_level + 1);
}

void MapHandle::printSearchParameters()
{
  std::cout << std::endl;
  std::cout << "Search parameters:" << std::endl;
  std::cout << "------------------" << std::endl;
  std::cout << "m_ground_search_level: " << m_ground_search_level << std::endl;
  std::cout << "closing_level: " << m_closing_level << std::endl;
  std::cout << "dilate_level: " << m_dilate_level << std::endl;
  std::cout << "step_height: " << m_step_height << std::endl;
  std::cout << "slope_box_range: " << m_slope_box_range << std::endl;
  std::cout << "max_slope_angle: " << m_max_slope_angle << std::endl;
  std::cout << "max_slope cells: " << m_max_slope_cell_count << std::endl;
}
