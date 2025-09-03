#include "DataNode.h"
#include <openvdb/openvdb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <type_traits>

#ifndef PLANNER_VDB_BASE_VDBVISUALIZATION_H_INCLUDED
#  define PLANNER_VDB_BASE_VDBVISUALIZATION_H_INCLUDED

namespace planner_vdb_base {


/**
 * \struct ColorRGB
 * \brief A struct for defining RGB color values.
 */
struct ColorRGB
{
  int r, g, b;
  ColorRGB(int r_, int g_, int b_)
    : r(r_)
    , g(g_)
    , b(b_)
  {
  }
};


/**
 * \brief Compatibility for occupancy visualization for non-DataNode grids.
 *
 * \tparam ValueType The value type of the grid.
 */
template <typename ValueType>
struct GridTypeHelper
{
  using DataNodeT      = ValueType;
  using DataNodeValueT = typename ValueType::DataType;
};

/**
 * \brief Compatibility for occupancy visualization for non-DataNode grids.
 */
template <>
struct GridTypeHelper<float>
{
  using DataNodeT      = std::nullptr_t;
  using DataNodeValueT = DataNode<std::nullptr_t>;
};

/**
 * \brief Compatibility for occupancy visualization for non-DataNode grids.
 */
template <>
struct GridTypeHelper<int32_t>
{
  using DataNodeT      = std::nullptr_t;
  using DataNodeValueT = DataNode<std::nullptr_t>;
};

/**
 * \class VDBVisualization
 *
 * \brief Collection of visualization tools for arithmetic values in DataNode and non DataNode VDB
 * grids.
 *
 * \tparam GridT The type of VDB grid to visualize.
 */
template <typename GridT>
class VDBVisualization
{
public:
  using RGBPointT      = pcl::PointXYZRGB;
  using RGBPointCloudT = pcl::PointCloud<RGBPointT>;

  // Compatibility for occupancy visualization for non-DataNode grids.
  using GridValueT     = typename GridT::ValueType;
  using DataNodeValueT = typename GridTypeHelper<typename GridT::ValueType>::DataNodeValueT;

  VDBVisualization(){};
  virtual ~VDBVisualization(){};

  // TODO adjust message and probably remove completely since it just calls one function
  /**
   * \brief Creates a pointcloud to visualize the voxel values of a openvdb built-in type grid.
   *
   * \param cloud_ptr The pointcloud to fill with the visualization.
   * \param grid The grid to visualize.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color map.
   */
  static void visualizeValueGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                 const typename GridT::Ptr grid,
                                 int hue_limit          = 255,
                                 bool positive_gradient = true);

  /**
   * \brief Creates a pointcloud to visualize the voxel values of a DataNode type grid.
   *
   * \param cloud_ptr The pointcloud to fill with the visualization.
   * \param grid The grid to visualize.
   * \param data_level The DataNode value to use for the visualization.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color map.
   */
  static void visualizeValueGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                 const typename GridT::Ptr grid,
                                 double DataNodeValueT::*data_level,
                                 int hue_limit          = 255,
                                 bool positive_gradient = true);

  /**
   * \brief Visualizes active voxels of a openvdb grid as a pointcloud.
   *
   * \param cloud_ptr The pointcloud to fill with the visualization.
   * \param grid The grid to visualize.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color map.
   */
  static void visualizeOccupiedGrid(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                                    const typename GridT::Ptr grid,
                                    int hue_limit          = 285,
                                    bool positive_gradient = false);

protected:
  /**
   * \brief Converts a value grid to a point cloud with RGB color values.
   *
   * \param cloud_ptr The point cloud to fill with the RGB color values.
   * \param grid The value grid to convert.
   * \param data_level The DataNode value to use for the visualization.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color
   * map.
   */
  static void valueMapToPCL(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                            const typename GridT::Ptr grid,
                            double DataNodeValueT::*data_level,
                            const int& hue_limit,
                            const bool& positive_gradient);

  /**
   * \brief Converts a value grid to a point cloud with RGB color values.
   *
   * \param cloud_ptr The point cloud to fill with the RGB color values.
   * \param grid The value grid to convert.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color
   * map.
   */
  static void valueMapToPCL(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                            const typename GridT::Ptr grid,
                            const int& hue_limit,
                            const bool& positive_gradient);

  /**
   * \brief Converts an occupied grid to a point cloud with RGB color values.
   *
   * \param cloud_ptr The point cloud to fill with the RGB color values.
   * \param grid The occupied grid to convert.
   * \param hue_limit The limit of the hue color map.
   * \param positive_gradient Whether to use a positive or negative gradient for the hue color
   */
  static void occupiedMapToPCL(std::shared_ptr<RGBPointCloudT>,
                               const typename GridT::Ptr grid,
                               const int& hue_limit,
                               const bool& positive_gradient);

  /**
   * \brief Gets the minimum and maximum values for a DataNode value in a grid.
   *
   * \param grid The grid to get the minimum and maximum values for.
   * \param data_level The DataNode value to get the minimum and maximum values for.
   * \param min
   * \param max_value The maximum value.
   */
  static void getMinMaxValue(const typename GridT::Ptr grid,
                             double DataNodeValueT::*data_level,
                             double& min_value,
                             double& max_value);

  /**
   * \brief Gets the minimum and maximum values for a non-DataNode value in a grid.
   *
   * \param grid The grid to get the minimum and maximum values for.
   * \param min_value The minimum value.
   * \param max_value The maximum value.
   */
  static void
  getMinMaxValue(const typename GridT::Ptr grid, GridValueT& min_value, GridValueT& max_value);


  /**
   * \brief Gets the minimum and maximum world heights of an occupied grid.
   *
   * \param grid The occupied grid to get the minimum and maximum world heights for.
   * \param min_z The minimum world height.
   * \param max_z The maximum world height.
   */
  static void getMinMaxWorldHeight(const typename GridT::Ptr grid, double& min_z, double& max_z);


  /**
   * \brief Adds an RGB point to a point cloud.
   *
   * \param cloud_ptr The point cloud to add the RGB point to.
   * \param world_coord The world coordinates of the RGB point.
   * \param color The RGB color of the point.
   */
  static void addRGBCloudPoint(std::shared_ptr<RGBPointCloudT> cloud_ptr,
                               openvdb::Vec3d world_coord,
                               ColorRGB color);

  /**
   * \brief Normalizes a value between 0 and 1 based on a minimum and maximum value.
   *
   * \param value The value to normalize.
   * \param min_value The minimum value to use for normalization.
   * \param max_value The maximum value to use for normalization.
   * \return The normalized value.
   */
  static double normalize(double value, const double& min_value, const double& max_value);

  /**
   * \brief Converts a hue value to an RGB color.
   *
   * \param hue_normed The normalized hue value.
   * \param hue_limit The limit of the hue color map.
   * \return The RGB color.
   */
  static ColorRGB hueColorCoding(const double& hue_normed, const int& hue_limit);
};
} // namespace planner_vdb_base

#  include "VDBVisualization.hpp"

#endif /* PLANNER_VDB_BASE_VDBVISUALIZATION_H_INCLUDED */
