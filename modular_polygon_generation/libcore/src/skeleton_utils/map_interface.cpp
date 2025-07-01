#include <libcore/skeleton_utils/map_interface.hpp>

namespace libcore
{
    pcl::PointCloud<pcl::PointXYZ> getMap() {
        pcl::PointCloud<pcl::PointXYZ> map;
        // Implementation to retrieve the map as a point cloud
        // This is a placeholder; actual implementation will depend on the specific requirements
        return map;
    }

    void preProcessMaps(pcl::PointCloud<pcl::PointXYZ> &map_pcl, 
                       pcl::PointCloud<pcl::PointXYZ> &raw_map_pcl,
                       pcl::PointCloud<pcl::PointXYZ> &vis_map_pcl,
                       const MapContext& context) {
        
        // Check if the map is empty before proceeding
        if (map_pcl.points.size() == 0) {
            std::cerr << "Error: Map is empty. Cannot proceed with pre-processing." << std::endl;
            return;
        }

        std::cout << "Point cloud size: " << map_pcl.points.size() << std::endl;
        // create a copy of the map for raw_map_pcl and vis_map_pcl by filtering all points where z > z_min
        for (const auto& point : map_pcl.points) {
            raw_map_pcl.points.push_back(point);
            if (point.z >= 0.2) vis_map_pcl.points.push_back(point);
        }

        // Add bounding box to the map
        addBbxToMap(map_pcl, context);
        addBbxToMap(raw_map_pcl, context);
    }

    void addBbxToMap(pcl::PointCloud<pcl::PointXYZ> &map, const MapContext& context) {
        double x_length = context.x_max - context.x_min;
        double y_length = context.y_max - context.y_min;
        // double z_length = _z_max - _z_min;
        int x_num = ceil(x_length / context.resolution) + 1;
        int y_num = ceil(y_length / context.resolution) + 1;
        // int z_num = ceil(z_length / _resolution) + 1;

        if (context.is_simulation) { // Add ceiling and floor to search map
            for (int i = 0; i < x_num; i++) {
            for (int j = 0; j < y_num; j++) {
                map.points.push_back(pcl::PointXYZ(context.x_min + context.resolution * i,
                                                context.y_min + context.resolution * j, context.z_min));
                map.points.push_back(pcl::PointXYZ(context.x_min + context.resolution * i,
                                                context.y_min + context.resolution * j, context.z_max));
            }
            }
        } else { // Add only ceiling to search map
            for (int i = 0; i < x_num; i++) {
            for (int j = 0; j < y_num; j++) {
                map.points.push_back(pcl::PointXYZ(context.x_min + context.resolution * i,
                                                context.y_min + context.resolution * j, context.z_max));
            }
            }
        }
    }
} // namespace libcore
