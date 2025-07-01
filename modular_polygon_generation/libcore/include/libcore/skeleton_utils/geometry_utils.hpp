#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Checks if a coordinate is within a bounding box.
     * 
     * This function checks if the given coordinate is within the specified bounding box defined by its minimum and maximum corners.
     * 
     * @param coord The coordinate to check.
     * @param bbx_min The minimum corner of the bounding box.
     * @param bbx_max The maximum corner of the bounding box.
     * @return true if the coordinate is within the bounding box, false otherwise.
     */
    bool checkWithinBbx(const Eigen::Vector3d& coord,
                        const Eigen::Vector3d& bbx_min,
                        const Eigen::Vector3d& bbx_max);

    
    bool checkFloor(NodePtr node, 
                    const double max_ray_length,
                    const double max_height_diff,
                    const RaycastingContext& ray_context);
    
    double radiusSearchOnRawMap(const Eigen::Vector3d& search_Pt,
                                const pcl::search::KdTree<pcl::PointXYZ>& kdtreeForRawMap);
    
    std::pair<double, int> radiusSearch(
        const Eigen::Vector3d& search_Pt,
        double search_margin,
        double max_ray_length,
        const std::vector<NodePtr>& NodeList,
        const pcl::search::KdTree<pcl::PointXYZ>& kdtreeForRawMap,
        const std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>>& kdtreesForPolys
    );

    double getDis(const NodePtr& node1, const NodePtr& node2);

    double getDis(const Eigen::Vector3d& coord1, const Eigen::Vector3d& coord2);

    double getDis(const Eigen::Vector3d& coord, const NodePtr& node);

    void centralizeNodePos(const NodePtr node);

    double getNodeRadius(const NodePtr& node);

    double getVerticesRadius(const std::vector<Eigen::Vector3d>& vertices);

    bool checkSegClear(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const RaycastingContext& ray_context
    );

    bool collisionCheck(
        const Eigen::Vector3d& search_pt,
        double threshold,
        const RaycastingContext& ray_context
    );

    bool isSamePos(
        const Eigen::Vector3d& coord1,
        const Eigen::Vector3d& coord2,
        double threshold = 1e-4
    );

} // namespace libcore