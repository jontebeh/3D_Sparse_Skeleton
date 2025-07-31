#include <libcore/skeleton_finder.hpp>
#include <libcore/skeleton_utils/map_interface.hpp>
#include <libcore/skeleton_utils/expansion.hpp>
#include <libcore/visualizer.hpp>
#include <iostream>
#include <chrono>
#include <libcore/ini/SimpleIni.h>

namespace libcore {
    SkeletonFinder::SkeletonFinder() {}

    SkeletonFinder::~SkeletonFinder() {}

    void SkeletonFinder::init(std::string config_file) {
        auto timer_1 = std::chrono::high_resolution_clock::now();
        std::cout << "Initializing SkeletonFinder with parameters." << std::endl;

        Config config = readConfigFile(config_file);
        std::cout << "config.max_ray_length: " << config.max_ray_length << std::endl;

        SharedVars vars(config);

        std::cout << "Setting up maps for skeleton finding." << std::endl;
        pcl::PointCloud<pcl::PointXYZ> map_pcl = getMap(config);
        pcl::PointCloud<pcl::PointXYZ> raw_map_pcl;
        auto timer_2 = std::chrono::high_resolution_clock::now();
        std::cout << "Timer 1: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(timer_2 - timer_1).count()
                  << " ms." << std::endl;

        std::cout << "Pre-processing maps for visualization and search." << std::endl;
        preProcessMaps(map_pcl, raw_map_pcl, config);

        auto timer_3 = std::chrono::high_resolution_clock::now();
        std::cout << "Timer 2: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(timer_3 - timer_2).count()
                  << " ms." << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB> map_pcl_rgb = getRGBMap(config);

        if (config.map_representation == 0) {
            vars.kdtreeForRawMap.setInputCloud(raw_map_pcl.makeShared());
        }

        vars.vis.InitWindow(map_pcl_rgb);

        auto timer_4 = std::chrono::high_resolution_clock::now();
        std::cout << "Timer 3: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(timer_4 - timer_3).count()
                  << " ms." << std::endl;

        std::cout << "Generating skeleton..." << std::endl;
        auto begin = std::chrono::high_resolution_clock::now();

        vars.startPt << config.start_x, config.start_y, config.start_z;
        std::thread compute_thread(
            [&vars, &config]() {
                std::cout << "Starting skeleton expansion..." << std::endl;
                skeletonExpansion(config, vars);
                std::cout << "Skeleton expansion completed." << std::endl;
            }
        );


        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        std::cout << "Skeleton generation completed in " << duration.count() << " ms." << std::endl;

        std::cout << "created " << vars.NodeList.size() << " nodes." << std::endl;
        std::cout << "created " << vars.center_NodeList.size() << " center nodes." << std::endl;

        // save node list to file
        std::ofstream node_file("node_list.txt");
        if (node_file.is_open()) {
            for (const auto& node : vars.NodeList) {
                node_file << node->id << ","
                          << node->coord.x() << ","
                          << node->coord.y() << ","
                          << node->coord.z() << "\n";
            }
            node_file.close();
            std::cout << "Node list saved to node_list.txt." << std::endl;
        } else {
            std::cerr << "Unable to open node_list.txt for writing." << std::endl;
        }

        // save edge list to file
        std::ofstream edge_file("edge_list.txt");
        if (edge_file.is_open()) {
            for (const auto& node : vars.NodeList) {
                for (const auto& connected_node : node->connected_nodes) {
                    edge_file << node->id << ","
                              << connected_node->id << "\n";
                }
            }
            edge_file.close();
            std::cout << "Edge list saved to edge_list.txt." << std::endl;
        } else {
            std::cerr << "Unable to open edge_list.txt for writing." << std::endl;
        }

        vars.vis.Run(); // This will block until the visualizer is closed
        compute_thread.join(); // Wait for the skeleton expansion to finish
        vars.vis.Close();

    }

    Config SkeletonFinder::readConfigFile(const std::string& config_file) {
        CSimpleIniA ini;
        ini.SetUnicode();
        std::string config_path = "../modular_polygon_generation/libcore/data/configs/" + config_file;
        if (ini.LoadFile(config_path.c_str()) < 0) {
            // throw an exception and terminate if the file cannot be loaded
            std::cerr << "Error loading configuration file: " << config_path << std::endl;
            throw std::runtime_error("Failed to load configuration file.");
        }
        CSimpleIniA::TNamesDepend sections;
        ini.GetAllSections(sections);

        for (const auto& section : sections) {
            CSimpleIniA::TNamesDepend keys;
            ini.GetAllKeys(section.pItem, keys);
            for (const auto& key : keys) {
                std::cout << "[" << section.pItem << "] "
                        << key.pItem << " = "
                        << ini.GetValue(section.pItem, key.pItem, "<not found>") << "\n";
            }
        }

        Config cfg;
        cfg.is_simulation = ini.GetBoolValue("Map", "is_simulation", cfg.is_simulation);
        cfg.x_min = ini.GetDoubleValue("Map", "x_min", cfg.x_min);
        cfg.x_max = ini.GetDoubleValue("Map", "x_max", cfg.x_max);
        cfg.y_min = ini.GetDoubleValue("Map", "y_min", cfg.y_min);
        cfg.y_max = ini.GetDoubleValue("Map", "y_max", cfg.y_max);
        cfg.z_min = ini.GetDoubleValue("Map", "z_min", cfg.z_min);
        cfg.z_max = ini.GetDoubleValue("Map", "z_max", cfg.z_max);
        cfg.map_representation = ini.GetLongValue("Map", "map_representation", cfg.map_representation);
        cfg.map = ini.GetValue("Map", "map_name", cfg.map.c_str());
        cfg.vis_map = ini.GetValue("Map", "vis_map_name", cfg.vis_map.c_str());


        cfg.resolution = ini.GetDoubleValue("Raycasting", "resolution", cfg.resolution);
        cfg.search_margin = ini.GetDoubleValue("Raycasting", "search_margin", cfg.search_margin);
        cfg.max_ray_length = ini.GetDoubleValue("Raycasting", "max_ray_length", cfg.max_ray_length);
        cfg.max_expansion_ray_length = ini.GetDoubleValue("Raycasting", "max_expansion_ray_length", cfg.max_expansion_ray_length);
        cfg.frontier_creation_threshold = ini.GetDoubleValue("Raycasting", "frontier_creation_threshold", cfg.frontier_creation_threshold);
        cfg.frontier_jump_threshold = ini.GetDoubleValue("Raycasting", "frontier_jump_threshold", cfg.frontier_jump_threshold);
        cfg.frontier_split_threshold = ini.GetDoubleValue("Raycasting", "frontier_split_threshold", cfg.frontier_split_threshold);
        cfg.max_height_diff = ini.GetDoubleValue("Raycasting", "max_height_diff", cfg.max_height_diff);
        cfg.min_node_radius = ini.GetDoubleValue("Raycasting", "min_node_radius", cfg.min_node_radius);
        cfg.min_flowback_creation_threshold = ini.GetLongValue("Raycasting", "min_flowback_creation_threshold", cfg.min_flowback_creation_threshold);
        cfg.min_flowback_creation_radius_threshold = ini.GetDoubleValue("Raycasting", "min_flowback_creation_radius_threshold", cfg.min_flowback_creation_radius_threshold);
        cfg.bad_loop_setting = ini.GetBoolValue("Raycasting", "bad_loop_setting", cfg.bad_loop_setting);
        cfg.max_facets_grouped = ini.GetLongValue("Raycasting", "max_facets_grouped", cfg.max_facets_grouped);

        cfg.start_x = ini.GetDoubleValue("Raycasting", "start_x", cfg.start_x);
        cfg.start_y = ini.GetDoubleValue("Raycasting", "start_y", cfg.start_y);
        cfg.start_z = ini.GetDoubleValue("Raycasting", "start_z", cfg.start_z);


        cfg.sampling_density = ini.GetLongValue("Sampling", "sampling_density", cfg.sampling_density);
        cfg.sampling_width = ini.GetDoubleValue("Sampling", "sampling_width", cfg.sampling_width);
        cfg.sampling_height = ini.GetDoubleValue("Sampling", "sampling_height", cfg.sampling_height);
        cfg.sampling_depth = ini.GetDoubleValue("Sampling", "sampling_depth", cfg.sampling_depth);
        cfg.sampling_sharpness = ini.GetDoubleValue("Sampling", "sampling_sharpness", cfg.sampling_sharpness);

        std::cout << "Config loaded from " << config_file << "\n";
        return cfg;
    }

}  // namespace libcore