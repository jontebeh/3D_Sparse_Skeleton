#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <libcore/data_types/node.hpp>
#include <libcore/visualizer.hpp>

namespace libcore
{
    struct Config
    {
        /*****************Map*****************/
        bool is_simulation = false; // Flag indicating if the context is for simulation.
        double resolution = 0.2; // The resolution of the map.
        double x_min = -24.0; // The minimum x-coordinate of the bounding box.
        double x_max = 85.0; // The minimum and maximum x-coordinates of the bounding box.
        double y_min = -18.0; // The minimum y-coordinate
        double y_max = 45.0; // The minimum and maximum y-coordinates of the bounding box.
        double z_min = 0.0; // The minimum z-coordinate
        double z_max = 13.0; // The maximum z-coordinate
        int map_representation = 0; // The representation of the map (e.g., point cloud, voxel grid, etc.).

        /*****************Raycasting*****************/
        double search_margin = 0.3; // Margin for searching in the map.
        double max_ray_length = 5.0; // Maximum length of the ray for raycasting.
        double max_expansion_ray_length = 4.0; // Maximum length for ray expansion.

        double frontier_creation_threshold = 0.3; // Threshold for creating frontiers.
        double frontier_jump_threshold = 2.0; // Threshold for jumping to a new frontier.
        double frontier_split_threshold = 0.5; // Threshold for splitting frontiers.

        int sampling_density = 100; // Density of the sampling used in the expansion.

        double max_height_diff = 2.5; // Maximum height difference allowed for floor checking.
        double min_node_radius = 1.0; // Minimum radius for nodes in the skeleton.

        int min_flowback_creation_threshold = 5; // Minimum threshold for creating flowback frontiers.
        double min_flowback_creation_radius_threshold = 0.5; // Minimum radius threshold for flowback creation.
        bool bad_loop_setting = true; // Flag to enable or disable bad loop settings.

        int max_facets_grouped = 10;

        double start_x = 45.0; // Starting x-coordinate for the skeleton expansion.
        double start_y = -16.0; // Starting y-coordinate for the skeleton expansion.
        double start_z = 7.0; // Starting z-coordinate for the skeleton expansion

        std::string map = "map.pcd"; // The name of the map file to be used.

        std::string vis_map = "vis_map.pcd"; // The name of the visualization map file.

        // default constructor
        Config() = default;
    };

    struct SharedVars
    {
        pcl::search::KdTree<pcl::PointXYZ> kdtreeForRawMap; // Kd-tree for raw map search.
        std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>> kdtreesForPolys; // Kd-trees for polygons.

        pcl::PointCloud<pcl::PointXYZ> nodes_pcl; // Point cloud for nodes in the skeleton.

        std::vector<NodePtr> NodeList; // List of nodes for raycasting operations.
        std::vector<NodePtr> center_NodeList; // List of center nodes for the skeleton.
        std::vector<Eigen::Vector3d> sample_directions; // Directions for sampling in the raycasting process.

        Eigen::Vector3d startPt; // The starting point for the skeleton expansion.
        Eigen::Vector3d bbx_min; // Minimum corner of the bounding box.
        Eigen::Vector3d bbx_max; // Maximum corner of the bounding box.

        std::deque<FrontierPtr> loop_candidate_frontiers; // List of candidate frontiers for loops.
        std::deque<FrontierPtr> pending_frontiers; // List of pending frontiers to be processed.

        std::vector<std::vector<Eigen::Vector3d>> bw_facets_directions; // Directions of black and white facets.
        
        Visualizer vis; // Visualizer for displaying the skeleton and other elements.

        // constructer with config
        SharedVars(const Config& config)
            : bbx_min(config.x_min, config.y_min, config.z_min),
              bbx_max(config.x_max, config.y_max, config.z_max)
        {}
    };
} // namespace libcore