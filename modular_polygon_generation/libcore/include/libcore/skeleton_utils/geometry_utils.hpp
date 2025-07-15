#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    bool checkWithinBbx(const Eigen::Vector3d& coord,
                        const Eigen::Vector3d& bbx_min,
                        const Eigen::Vector3d& bbx_max);

    bool checkFloor(
        const NodePtr node, 
        const double max_ray_length,
        const double max_height_diff,
        const Config& config, SharedVars& vars
    );
    
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
        const Eigen::Vector3d& pos1,
        const Eigen::Vector3d& pos2,
        const Config& config,
        SharedVars& vars
    );

    bool collisionCheck(
        const Eigen::Vector3d& search_pt,
        double threshold,
        const Config& config,
        SharedVars& vars
    );

    bool isSamePos(
        const Eigen::Vector3d& coord1,
        const Eigen::Vector3d& coord2,
        double threshold = 1e-4
    );

    VertexPtr getVertexFromDire(
        NodePtr node,
        const Eigen::Vector3d &dire
    );

    int onCeilOrFloor(
        const Eigen::Vector3d p,
        const double z_min,
        const double z_max,
        const double search_margin
    );

    bool checkPtInPolyhedron(
        const NodePtr node, 
        const Eigen::Vector3d pt
    );

} // namespace libcore