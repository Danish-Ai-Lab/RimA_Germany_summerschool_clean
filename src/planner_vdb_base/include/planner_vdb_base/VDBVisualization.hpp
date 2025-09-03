#include "planner_vdb_base/VDBVisualization.h"

namespace planner_vdb_base {

template <typename GridT>
void VDBVisualization<GridT>::valueMapToPCL(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                            const typename GridT::Ptr grid,
                                            double DataNodeValueT::*data_level,
                                            const int& hue_limit,
                                            const bool& positive_gradient)
{
  typename GridT::Accessor value_grid_acc = grid->getAccessor();
  openvdb::Vec3d world_coord;
  double min_value, max_value;
  getMinMaxValue(grid, data_level, min_value, max_value);
  for (typename GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    GridValueT voxel_data_node = value_grid_acc.getValue(iter.getCoord());
    DataNodeValueT voxel_data  = voxel_data_node.getData();

    double value        = voxel_data.*data_level;
    double value_normed = normalize(value, min_value, max_value);

    if (positive_gradient == false)
    {
      value_normed = 1.0 - value_normed;
    }
    world_coord    = grid->indexToWorld(iter.getCoord());
    ColorRGB color = hueColorCoding(value_normed, hue_limit);

    addRGBCloudPoint(cloud_ptr, world_coord, color);
  }
  cloud_ptr->width  = cloud_ptr->points.size();
  cloud_ptr->height = 1;
}

template <typename GridT>
void VDBVisualization<GridT>::valueMapToPCL(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                            const typename GridT::Ptr grid,
                                            const int& hue_limit,
                                            const bool& positive_gradient)
{
  typename GridT::Accessor value_grid_acc = grid->getAccessor();
  openvdb::Vec3d world_coord;
  GridValueT min_value, max_value;
  getMinMaxValue(grid, min_value, max_value);
  for (typename GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    GridValueT voxel_value = value_grid_acc.getValue(iter.getCoord());
    double value_normed    = normalize(voxel_value, min_value, max_value);

    if (positive_gradient == false)
    {
      value_normed = 1.0 - value_normed;
    }
    world_coord    = grid->indexToWorld(iter.getCoord());
    ColorRGB color = hueColorCoding(value_normed, hue_limit);

    addRGBCloudPoint(cloud_ptr, world_coord, color);
  }
  cloud_ptr->width  = cloud_ptr->points.size();
  cloud_ptr->height = 1;
}

template <typename GridT>
void VDBVisualization<GridT>::occupiedMapToPCL(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                               const typename GridT::Ptr grid,
                                               const int& hue_limit,
                                               const bool& positive_gradient)
{
  double min_z, max_z;
  getMinMaxWorldHeight(grid, min_z, max_z);

  openvdb::Vec3d world_coord;

  for (typename GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    world_coord         = grid->indexToWorld(iter.getCoord());
    double value_normed = normalize(world_coord.z(), min_z, max_z);
    if (positive_gradient == false)
    {
      value_normed = 1.0 - value_normed;
    }
    ColorRGB color = hueColorCoding(value_normed, hue_limit);
    addRGBCloudPoint(cloud_ptr, world_coord, color);
  }
  cloud_ptr->width  = cloud_ptr->points.size();
  cloud_ptr->height = 1;
}

template <typename GridT>
void VDBVisualization<GridT>::getMinMaxWorldHeight(const typename GridT::Ptr grid,
                                                   double& min_z,
                                                   double& max_z)
{
  openvdb::CoordBBox bbox        = grid->evalActiveVoxelBoundingBox();
  openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
  openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());
  min_z                          = min_world_coord.z();
  max_z                          = max_world_coord.z();
}

template <typename GridT>
void VDBVisualization<GridT>::getMinMaxValue(const typename GridT::Ptr grid,
                                             double DataNodeValueT::*data_level,
                                             double& min_value,
                                             double& max_value)
{
  typename GridT::Accessor value_grid_acc = grid->getAccessor();
  min_value                               = std::numeric_limits<double>::max();
  max_value                               = std::numeric_limits<double>::min();
  for (typename GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    GridValueT voxel_data_node = value_grid_acc.getValue(iter.getCoord());
    DataNodeValueT voxel_data  = voxel_data_node.getData();
    double value               = voxel_data.*data_level;

    if (std::abs(value) == std::numeric_limits<double>::infinity())
    {
      continue;
    }

    min_value = std::min(min_value, (double)value);
    max_value = std::max(max_value, (double)value);
  }
}

template <typename GridT>
void VDBVisualization<GridT>::getMinMaxValue(const typename GridT::Ptr grid,
                                             GridValueT& min_value,
                                             GridValueT& max_value)
{
  typename GridT::Accessor value_grid_acc = grid->getAccessor();
  min_value                               = std::numeric_limits<GridValueT>::max();
  max_value                               = std::numeric_limits<GridValueT>::min();
  for (typename GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    GridValueT voxel_value = value_grid_acc.getValue(iter.getCoord());

    if (std::abs(voxel_value) == std::numeric_limits<GridValueT>::infinity())
    {
      continue;
    }

    min_value = std::min(min_value, voxel_value);
    max_value = std::max(max_value, voxel_value);
  }
}

template <typename GridT>
void VDBVisualization<GridT>::addRGBCloudPoint(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                               openvdb::Vec3d world_coord,
                                               ColorRGB color)
{
  RGBPointT point;
  point.x = world_coord.x();
  point.y = world_coord.y();
  point.z = world_coord.z();
  point.r = color.r;
  point.g = color.g;
  point.b = color.b;
  cloud_ptr->points.push_back(point);
}


template <typename GridT>
void VDBVisualization<GridT>::visualizeValueGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                                 const typename GridT::Ptr grid,
                                                 int hue_limit,
                                                 bool positive_gradient)
{
  valueMapToPCL(cloud_ptr, grid, hue_limit, positive_gradient);
}

template <typename GridT>
void VDBVisualization<GridT>::visualizeValueGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                                 const typename GridT::Ptr grid,
                                                 double DataNodeValueT::*data_level,
                                                 int hue_limit,
                                                 bool positive_gradient)
{
  valueMapToPCL(cloud_ptr, grid, data_level, hue_limit, positive_gradient);
}

template <typename GridT>
void VDBVisualization<GridT>::visualizeOccupiedGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                                    const typename GridT::Ptr grid,
                                                    int hue_limit,
                                                    bool positive_gradient)
{
  occupiedMapToPCL(cloud_ptr, grid, hue_limit, positive_gradient);
}

template <typename GridT>
double
VDBVisualization<GridT>::normalize(double value, const double& min_value, const double& max_value)
{
  if (min_value == max_value)
  {
    throw std::invalid_argument("Min and max values cannot be the same.");
  }

  if (value > max_value)
  {
    value = max_value;
  }
  else if (value < min_value)
  {
    value = min_value;
  }

  return (value - min_value) / (max_value - min_value);
}

template <typename GridT>
ColorRGB VDBVisualization<GridT>::hueColorCoding(const double& hue_normed, const int& hue_limit)
{
  double h = hue_normed * ((double)hue_limit / 360.0);

  int i    = (int)(h * 6.0);
  double f = (h * 6.0) - i;
  double q = (1.0 - f);
  i %= 6;

  switch (i)
  {
    case 0:
      return ColorRGB(255, f * 255, 0);
      break;
    case 1:
      return ColorRGB(q * 255, 255, 0);
      break;
    case 2:
      return ColorRGB(0, 255, f * 255);
      break;
    case 3:
      return ColorRGB(0, q * 255, 255);
      break;
    case 4:
      return ColorRGB(f * 255, 0, 255);
      break;
    case 5:
      return ColorRGB(255, 0, q * 255);
      break;
    default:
      return ColorRGB(255, 255, 255);
  }
}
} // namespace planner_vdb_base
