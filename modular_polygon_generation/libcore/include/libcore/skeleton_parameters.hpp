#pragma once

#include <Eigen/Dense>

namespace libcore {

    /**
     * @brief Parameters for skeleton finding algorithm.
     */
    struct SkeletonParameters {
        // Map representation
        // 0: point cloud; 1: occupancy map
        int map_representation;
        // Whether the map is in simulation
        bool is_simulation;
        // An edge will be considered as a frontier if:
        // the dist to its nearest point exceeds this threshold
        double frontier_creation_threshold;
        // Jump frontier
        double frontier_jump_threshold;
        // Facets will be split into diff frontier if the angle between exceeds this threshold
        double frontier_split_threshold;
        // A flowback will be created if number of contact vertices exceeds this threshold
        int min_flowback_creation_threshold;
        // A flowback will not be created if the radius of contact vertices is below this threshold
        double min_flowback_creation_radius_threshold;
        // A node will be discarded if its average vertex-center distance is below
        // this threshold
        double min_node_radius;
        // A point on the ray will be considered as hit the pcl if:
        // the dist to its nearest point is below this margin
        // search_margin > sqrt((resolution/2)^2 + (raycast_step/2)^2)
        double search_margin;
        // A ray will be discarded if length exceeds this max
        double max_ray_length;
        // A new node will be set at the midpoint if length exceeds this max
        double max_expansion_ray_length;
        // A node will be absorbed if difference of distance to floor with its parent exceeds this limit
        double max_height_diff;
        // Number of sampings on the unit sphere
        int sampling_density;
        // Max number of facets grouped in a frontier
        int max_facets_grouped;
        // Resolution for map, raycast,
        double resolution;
        // Visualization
        double truncated_z_high;
        double truncated_z_low;
        // Bounding box
        double x_min, x_max, y_min, y_max, z_min, z_max;
        double start_x, start_y, start_z;
        double path_start_x, path_start_y, path_start_z;
        double path_target_x, path_target_y, path_target_z;

        /* ------------------ Development Tune ------------------ */
        bool debug_mode;
        bool bad_loop;

        // Visualize only the final result or the expansion process
        bool visualize_final_result_only;
        // Visualize all or only the newest polyhedron
        bool visualize_all;
        // Visualize outwards normal for each frontier
        bool visualize_outwards_normal;
        // Visualize neighborhood facets for each frontier
        bool visualize_nbhd_facets;
        // Visualize only_black polygon or black_and_white polygon
        bool visualize_black_polygon;
    };
} // namespace libcore