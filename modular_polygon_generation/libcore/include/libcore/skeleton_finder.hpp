#pragma once

#include <libcore/skeleton_parameters.hpp>
#include <libcore/data_types/graph_types.hpp>
#include <libcore/data_types/contexts.hpp>
#include <libcore/skeleton_utils/map_interface.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace libcore {

class SkeletonFinder {
public:
    /**
     * @brief Default constructor for SkeletonFinder.
     */
    SkeletonFinder();

    /**
     * @brief Destructor for SkeletonFinder.
     */
    ~SkeletonFinder();

    /**
     * @brief Initializes the skeleton finder with the given parameters.
     * 
     * @param params The parameters for the skeleton finding algorithm.
     */
    void init(const SkeletonParameters& params);

    /**
     * @brief Gets the skeleton graph after the algorithm has been run.
     * @return The skeleton graph as a Skeleton object.
     */
    Skeleton getSkeletonGraph() const;

private:
    SkeletonParameters _params;

    /*-----------------Map------------------*/
    // Map used in the main search process,
    // including obstacles, bounding box(optional), polyhedron facets
    pcl::PointCloud<pcl::PointXYZ> map_pcl;
    // Map used in closing loops,
    // including obstacles, bounding box
    pcl::PointCloud<pcl::PointXYZ> raw_map_pcl;
    // Map used in visualization,
    // including obstacles, excluding bounding box
    pcl::PointCloud<pcl::PointXYZ> vis_map_pcl;

    /*-------------KD-Trees-------------------*/
    std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>> kdtreesForPolys; // KD-Trees for polygons
    pcl::search::KdTree<pcl::PointXYZ> kdtreeForRawMap; // KD-Tree for raw map
    pcl::search::KdTree<pcl::PointXYZ> kdtreeForVisMap;  // KD-Tree for visualization map
    pcl::search::KdTree<pcl::PointXYZ> kdtreeForTestMap;  // KD-Tree for test map
    pcl::search::KdTree<pcl::PointXYZ> kdtreeForNodes; // KD-Tree for nodes



    /**
     * @brief Initializes the skeleton finder with parameters from the node handle.
     * 
     * @param params The parameters for the skeleton finding algorithm.
     */
    void setParameters(const SkeletonParameters& params);

    /**
     * @brief Starts the skeleton finding algorithm.
     * 
     * @param startPt The starting point coordinates as an Eigen::Vector3d.
     */
    void skeletonExpension(Eigen::Vector3d startPt);
};

} // namespace libcore
