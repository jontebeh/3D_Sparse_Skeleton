#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Gets the map as a point cloud.
     * This function retrieves the current map represented as a pcl::PointCloud<pcl::PointXYZ>.
     * @return pcl::PointCloud<pcl::PointXYZ> The point cloud representing the map.
     */
    pcl::PointCloud<pcl::PointXYZ> getMap();

    /**
     * @brief Pre-processes the maps for visualization and search.
     * This function prepares the maps by applying necessary transformations or filtering
     * to make them suitable for visualization and search operations.
     * @param map_pcl The point cloud representing the main map.
     * @param raw_map_pcl The point cloud representing the raw map.
     * @param vis_map_pcl The point cloud for visualization purposes.
     * @param context The context containing parameters for pre-processing.
     */
    void preProcessMaps(pcl::PointCloud<pcl::PointXYZ> &map_pcl, 
                       pcl::PointCloud<pcl::PointXYZ> &raw_map_pcl,
                       pcl::PointCloud<pcl::PointXYZ> &vis_map_pcl,
                       const MapContext& context);

    /**
     * @brief Adds a bounding box to the map.
     * This function adds a bounding box to the current map represented as a pcl::PointCloud<pcl::PointXYZ>.
     * @param map The point cloud representing the map to which the bounding box will be added.
     * @param is_simulation A boolean indicating whether the operation is in simulation mode (default is false).
     */
    void addBbxToMap(pcl::PointCloud<pcl::PointXYZ> &map, const MapContext& context);

} // namespace libcore