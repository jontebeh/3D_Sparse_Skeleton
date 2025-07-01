#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/node.hpp>

namespace libcore
{
    /**
     * @brief Context for the map, including simulation mode and bounding box dimensions.
     * 
     * This struct holds the context information for the map, including whether it is in simulation mode,
     * the resolution of the map, and the bounding box dimensions (x_min, x_max, y_min, y_max, z_min, z_max).
     */
    struct MapContext
    {
        bool is_simulation; // Flag indicating if the context is for simulation.
        double resolution; // The resolution of the map.
        double x_min, x_max; // The minimum and maximum x-coordinates of the bounding box.
        double y_min, y_max; // The minimum and maximum y-coordinates of the bounding box.
        double z_min, z_max; // The minimum and maximum z-coordinates of the bounding box.
        int map_representation; // The representation of the map (e.g., point cloud, voxel grid, etc.).

        /**
         * @brief Default constructor for MapContext.
         * 
         * Initializes the map context with default values.
         */
        MapContext()
            : is_simulation(false),
              resolution(0.0),
              x_min(0.0), x_max(0.0),
              y_min(0.0), y_max(0.0),
              z_min(0.0), z_max(0.0),
              map_representation(0) {} // Default representation, e.g., point cloud

        /**
         * @brief Constructor for MapContext.
         * 
         * Initializes the map context with the given parameters.
         * 
         * @param is_simulation Flag indicating if the context is for simulation.
         * @param resolution The resolution of the map.
         * @param x_min Minimum x-coordinate of the bounding box.
         * @param x_max Maximum x-coordinate of the bounding box.
         * @param y_min Minimum y-coordinate of the bounding box.
         * @param y_max Maximum y-coordinate of the bounding box.
         * @param z_min Minimum z-coordinate of the bounding box.
         * @param z_max Maximum z-coordinate of the bounding box.
         * @param map_representation Representation of the map.
         */
        MapContext(bool is_simulation, double resolution,
                   double x_min, double x_max,
                   double y_min, double y_max,
                   double z_min, double z_max,
                   int map_representation)
            : is_simulation(is_simulation),
              resolution(resolution),
              x_min(x_min), x_max(x_max),
              y_min(y_min), y_max(y_max),
              z_min(z_min), z_max(z_max),
              map_representation(map_representation) {}
    };

    /**
     * @brief Context for raycasting operations.
     */
    struct RaycastingContext
    {
        double search_margin;        // Margin for searching in the map.
        const pcl::search::KdTree<pcl::PointXYZ> &kdtreeForRawMap; // Kd-tree pointer for raw map search.
        const std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>> kdtreesForPolys; // Kd-trees for polygons.
        double max_ray_length; // Maximum length of the ray for raycasting.
        std::vector<NodePtr>& NodeList; // List of nodes for raycasting operations.
        const MapContext& map_context; // Context for the map.

        /**
         * @brief Default constructor for RaycastingContext.
         * 
         * Initializes the raycasting context with default values.
         */
        RaycastingContext()
            : map_representation(0), // Default representation, e.g., point cloud
              search_margin(0.0),
              kdtreeForRawMap(*(new pcl::search::KdTree<pcl::PointXYZ>())) // Default empty Kd-tree
        {}

        /**
         * @brief Constructor for RaycastingContext.
         * 
         * Initializes the raycasting context with the given parameters.
         * 
         * @param map_representation Representation of the map.
         * @param search_margin Margin for searching in the map.
         * @param kdtreeForRawMap Kd-tree pointer for raw map search.
         */
        RaycastingContext(
            int map_representation, 
            double search_margin,
            const pcl::search::KdTree<pcl::PointXYZ> &kdtreeForRawMap)
            : map_representation(map_representation),
              search_margin(search_margin),
              kdtreeForRawMap(kdtreeForRawMap) {}
    };

    struct SamplingContext {
        std::vector<Eigen::Vector3d>& sample_directions; // Directions for sampling.
        int sampling_density; // Density of the sampling.
        const RaycastingContext& ray_context; // Context for raycasting operations.


        /**
         * @brief Default constructor for SamplingContext.
         * 
         * Initializes the sampling context with an empty vector for sample directions.
         */
        SamplingContext() : 
            sample_directions(*(new std::vector<Eigen::Vector3d>())), // Default empty vector
            sampling_density(0), // Default sampling density
            ray_context(*(new RaycastingContext())) // Default empty RaycastingContext
        {}
    };

    /**
     * @brief Context for the skeleton expansion algorithm.
     * 
     * This struct holds the context information for the skeleton expansion algorithm.
     */
    struct ExpansionContext {
        Eigen::Vector3d startPt; // The starting point for the skeleton expansion.
        int sampling_density; // The density of the sampling used in the expansion.
        Eigen::Vector3d bbx_min; // Minimum corner of the bounding box.
        Eigen::Vector3d bbx_max; // Maximum corner of the bounding box.
        double max_height_diff; // Maximum allowed height difference for floor checking.
        double min_node_radius; // Minimum radius for nodes in the skeleton.
        std::deque<FrontierPtr>& loop_candidate_frontiers; // List of candidate frontiers for loops.
        int min_flowback_creation_threshold; // Minimum threshold for creating flowback frontiers.
        double min_flowback_creation_radius_threshold; // Minimum radius threshold for flowback creation.
        bool bad_loop_setting; // Flag to enable or disable bad loop settings.
        std::vector<NodePtr>& center_NodeList;
        pcl::PointCloud<pcl::PointXYZ>& nodes_pcl;

        const RaycastingContext& ray_context; // Context for raycasting operations.

        const SamplingContext& sampling_context; // Context for sampling operations.

        /**
         * @brief Default constructor for ExpansionContext.
         * 
         * Initializes the expansion context with default values.
         */
        ExpansionContext()
            : startPt(Eigen::Vector3d::Zero()),
              sampling_density(0),
              bbx_min(Eigen::Vector3d::Zero()),
              bbx_max(Eigen::Vector3d::Zero()),
              max_ray_length(0.0),
              max_height_diff(0.0),
              ray_context(*(new RaycastingContext())), // Default empty RaycastingContext
              sampling_context(*(new SamplingContext())) // Default empty SamplingContext

        {}

        /**
         * @brief Constructor for ExpansionContext.
         * 
         * Initializes the expansion context with the given parameters.
         * 
         * @param startPt Starting point for the skeleton expansion.
         * @param sampling_density Density of the sampling used in the expansion.
         * @param bbx_min Minimum corner of the bounding box.
         * @param bbx_max Maximum corner of the bounding box.
         * @param max_ray_length Maximum length of the ray for raycasting.
         * @param max_height_diff Maximum allowed height difference for floor checking.
         * @param ray_context Context for raycasting operations.
         */
        ExpansionContext(
            const Eigen::Vector3d& startPt, 
            int sampling_density,
            const Eigen::Vector3d& bbx_min, 
            const Eigen::Vector3d& bbx_max,
            double max_ray_length, 
            double max_height_diff,
            const RaycastingContext& ray_context,
            const SamplingContext& sampling_context)
            :   startPt(startPt),
                sampling_density(sampling_density),
                bbx_min(bbx_min),
                bbx_max(bbx_max),
                max_ray_length(max_ray_length),
                max_height_diff(max_height_diff),
                ray_context(ray_context),
                sampling_context(sampling_context)
        {}
    };


} // namespace libcore