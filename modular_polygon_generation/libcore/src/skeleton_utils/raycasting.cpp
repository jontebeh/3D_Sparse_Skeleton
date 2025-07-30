#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

namespace libcore {
    std::pair<Eigen::Vector3d, int> raycastOnRawMap(
        Eigen::Vector3d& ray_source,
        Eigen::Vector3d& direction,
        double cut_off_length,
        const Config& config,
        SharedVars& vars)
    {
        if (config.map_representation != 0) { // not a point cloud
            return std::pair<Eigen::Vector3d, int>(Eigen::Vector3d::Zero(), 0);
        }

        double clearance = radiusSearchOnRawMap(ray_source, vars.kdtreeForRawMap); // Distance to the nearest point
        if (clearance > cut_off_length) { // If the nearest point is farther than the cut-off length
            return std::pair<Eigen::Vector3d, int>(ray_source, -2);
        }

        Eigen::Vector3d current_pos = ray_source + clearance * direction; // Start from the point after the clearance
        double length = clearance; // total length of the ray

        while (length <= cut_off_length) { // While the ray length is within the cut-off length
            double radius = radiusSearchOnRawMap(current_pos, vars.kdtreeForRawMap); // Distance to the nearest point from the current position

            if (radius < config.search_margin) { // If the distance to the nearest point is less than the search margin return the current position with the flag -1 meaning a point was found
                return std::pair<Eigen::Vector3d, int>(current_pos, -1);
            }

            current_pos += radius * direction; // Move to the next point along the ray
            length += radius; // Update the total length of the ray
        }

        return std::pair<Eigen::Vector3d, int>(ray_source, -2); // If no point was found within the cut-off length, return the original ray source with the flag -2 meaning no point was found
    }

    std::pair<Eigen::Vector3d, int> raycast(
        Eigen::Vector3d ray_source,
        Eigen::Vector3d direction,
        double cut_off_length,
        const Config& config,
        SharedVars& vars)
    {
        double clearance = radiusSearch(
            ray_source,
            config.search_margin,
            config.max_ray_length,
            vars.NodeList,
            vars.kdtreeForRawMap,
            vars.kdtreesForPolys
        ).first;
        
        if (clearance > cut_off_length) {
            std::pair<Eigen::Vector3d, int> return_pair(ray_source, -2);
            return return_pair;
        } else {
            Eigen::Vector3d current_pos = ray_source + clearance * direction;
            double length = clearance;
            while (length <= cut_off_length) {
                std::pair<double, int> rs = radiusSearch(
                    current_pos,
                    config.search_margin,
                    config.max_ray_length,
                    vars.NodeList,
                    vars.kdtreeForRawMap,
                    vars.kdtreesForPolys
                );
                double radius = rs.first;

                if (radius < config.search_margin) {
                    return std::make_pair(current_pos, rs.second);
                }
                current_pos += radius * direction;
                length += radius;
            }

            return std::make_pair(ray_source, -2);
        }
    }
} // namespace libcore