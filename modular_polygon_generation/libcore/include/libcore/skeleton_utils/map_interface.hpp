#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Retrieves the map as a PointCloud from the specified configuration.
     * 
     * @param config The configuration containing map parameters.
     * @return pcl::PointCloud<pcl::PointXYZ> The point cloud representing the map.
     */
    pcl::PointCloud<pcl::PointXYZ> getMap(Config &config);

    pcl::PointCloud<pcl::PointXYZRGB> getRGBMap(Config& config);

    void preProcessMaps(
        pcl::PointCloud<pcl::PointXYZ> &map_pcl, 
        pcl::PointCloud<pcl::PointXYZ> &raw_map_pcl,
        const Config& config
    );

    void addBbxToMap(
        pcl::PointCloud<pcl::PointXYZ> &map, 
        const Config& config
    );

} // namespace libcore