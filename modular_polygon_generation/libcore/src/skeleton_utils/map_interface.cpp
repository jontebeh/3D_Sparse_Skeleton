#include <libcore/skeleton_utils/map_interface.hpp>
#include <pcl/io/pcd_io.h>

namespace libcore
{
    pcl::PointCloud<pcl::PointXYZ> getMap(Config& config) {
        pcl::PointCloud<pcl::PointXYZ> map;
        // read pcl from file
        std::string map_file = "../modular_polygon_generation/libcore/data/maps/" + config.map;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, map) == -1) {
            PCL_ERROR("Couldn't read the map file \n");
            return map; // Return an empty map if loading fails
        }

        return map;
    }

    pcl::PointCloud<pcl::PointXYZRGB> getRGBMap(Config& config) {
        pcl::PointCloud<pcl::PointXYZRGB> map;
        // read pcl from file
        std::string map_file = "../modular_polygon_generation/libcore/data/maps/" + config.vis_map;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_file, map) == -1) {
            PCL_ERROR("Couldn't read the map file \n");
            return map; // Return an empty map if loading fails
        }

        return map;
    }

    void preProcessMaps(
        pcl::PointCloud<pcl::PointXYZ> &map_pcl, 
        pcl::PointCloud<pcl::PointXYZ> &raw_map_pcl,
        const Config& config
    ) {
        
        // Check if the map is empty before proceeding
        if (map_pcl.points.size() == 0) {
            std::cerr << "Error: Map is empty. Cannot proceed with pre-processing." << std::endl;
            return;
        }

        std::cout << "Point cloud size: " << map_pcl.points.size() << std::endl;
        // create a copy of the map for raw_map_pcl and vis_map_pcl by filtering all points where z > z_min
        for (const auto& point : map_pcl.points) {
            raw_map_pcl.points.push_back(point);
        }

        // Add bounding box to the map
        addBbxToMap(map_pcl, config);
        addBbxToMap(raw_map_pcl, config);
    }

    void addBbxToMap(
        pcl::PointCloud<pcl::PointXYZ> &map, 
        const Config& config
    ) {
        double x_length = config.x_max - config.x_min;
        double y_length = config.y_max - config.y_min;
        int x_num = ceil(x_length / config.resolution) + 1;
        int y_num = ceil(y_length / config.resolution) + 1;

        if (config.is_simulation) { // Add ceiling and floor to search map
            for (int i = 0; i < x_num; i++) {
            for (int j = 0; j < y_num; j++) {
                map.points.push_back(pcl::PointXYZ(config.x_min + config.resolution * i,
                                                   config.y_min + config.resolution * j, config.z_min));
                map.points.push_back(pcl::PointXYZ(config.x_min + config.resolution * i,
                                                   config.y_min + config.resolution * j, config.z_max));
            }
            }
        } else { // Add only ceiling to search map
            for (int i = 0; i < x_num; i++) {
            for (int j = 0; j < y_num; j++) {
                map.points.push_back(pcl::PointXYZ(config.x_min + config.resolution * i,
                                                   config.y_min + config.resolution * j, config.z_max));
            }
            }
        }
    }
} // namespace libcore
