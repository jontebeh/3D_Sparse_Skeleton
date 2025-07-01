#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

namespace libcore {
    std::pair<Eigen::Vector3d, int> raycastOnRawMap(
        Eigen::Vector3d& ray_source,
        Eigen::Vector3d& direction,
        double cut_off_length,
        const RaycastingContext& c
        )
    {
        // Point cloud map
        if (c.map_representation == 0) {
            double clearance = radiusSearchOnRawMap(ray_source, c.kdtreeForRawMap);
            if (clearance > cut_off_length) {
                return std::pair<Eigen::Vector3d, int>(ray_source, -2);
            } else {
                Eigen::Vector3d current_pos = ray_source + clearance * direction;
                double length = clearance;

                while (length <= cut_off_length) {
                    double radius = radiusSearchOnRawMap(current_pos, c.kdtreeForRawMap);

                    if (radius < c.search_margin) {
                        return std::pair<Eigen::Vector3d, int>(current_pos, -1);
                    }
                    current_pos += radius * direction;
                    length += radius;
                }

                return std::pair<Eigen::Vector3d, int>(ray_source, -2);
            }
        }
        // Won't reach
        return std::pair<Eigen::Vector3d, int>(Eigen::Vector3d::Zero(), 0);
    }

    std::pair<Eigen::Vector3d, int> raycast(
        Eigen::Vector3d ray_source,
        Eigen::Vector3d direction,
        double cut_off_length,
        const RaycastingContext& c
    ) 
    {
        double clearance = radiusSearch(
            ray_source,
            c.search_margin,
            c.max_ray_length,
            c.NodeList,
            c.kdtreeForRawMap,
            c.kdtreesForPolys
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
                    c.search_margin,
                    c.max_ray_length,
                    c.NodeList,
                    c.kdtreeForRawMap,
                    c.kdtreesForPolys
                );
                double radius = rs.first;

                if (radius < c.search_margin) {
                    return std::make_pair(current_pos, rs.second);
                }
                current_pos += radius * direction;
                length += radius;
            }

            return std::make_pair(ray_source, -2);
        }
    }
} // namespace libcore