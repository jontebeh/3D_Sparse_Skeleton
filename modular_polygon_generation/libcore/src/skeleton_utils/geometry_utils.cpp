#include <libcore/skeleton_utils/geometry_utils.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>


namespace libcore
{
    bool checkWithinBbx(const Eigen::Vector3d& coord,
                        const Eigen::Vector3d& bbx_min,
                        const Eigen::Vector3d& bbx_max) {
        return (coord(0) >= bbx_min(0) && coord(1) >= bbx_min(1) && coord(2) >= bbx_min(2) &&
                coord(0) <= bbx_max(0) && coord(1) <= bbx_max(1) && coord(2) <= bbx_max(2));
    }

    bool checkFloor(const NodePtr node, 
                    const double max_ray_length,
                    const double max_height_diff,
                    const RaycastingContext& ray_context) {
        Eigen::Vector3d downwards(0, 0, -1);
        std::pair<Eigen::Vector3d, int> raycast_result =
            raycastOnRawMap(node->coord, downwards, max_ray_length, ray_context);
    
        if (raycast_result.second == -2) return false;
        double floor_height = raycast_result.first(2);

        // First node case
        if (node->seed_frontier == NULL) {
            node->dis_to_floor = floor_height;
            return true;
        }

        if (!node->isGate) {
            Eigen::Vector3d mid = (node->coord + node->seed_frontier->proj_center) / 2;
            std::pair<Eigen::Vector3d, int> raycast_result_mid =
                raycastOnRawMap(mid, downwards, max_ray_length, ray_context);
            if (raycast_result_mid.second == -2) return false;

            Eigen::Vector3d mid2 = (node->seed_frontier->proj_center +
                                    node->seed_frontier->master_node->coord) / 2;
            std::pair<Eigen::Vector3d, int> raycast_result_mid2 =
                raycastOnRawMap(mid2, downwards, max_ray_length, ray_context);
            if (raycast_result_mid2.second == -2) return false;
        }

        double parent_floor_height = node->seed_frontier->master_node->dis_to_floor;
        if (fabs(floor_height - parent_floor_height) > max_height_diff) return false;

        node->dis_to_floor = floor_height;

        return true;
    }

    double radiusSearchOnRawMap(const Eigen::Vector3d& search_Pt, 
                                const pcl::search::KdTree<pcl::PointXYZ>& kdtreeForRawMap) {
        pcl::PointXYZ searchPoint;
        searchPoint.x = search_Pt(0);
        searchPoint.y = search_Pt(1);
        searchPoint.z = search_Pt(2);

        std::vector<int> indices(1);
        std::vector<float> distance(1);

        kdtreeForRawMap.nearestKSearch(searchPoint, 1, indices, distance);
        double radius = sqrt(indices[0]);
        return radius;
    }

    std::pair<double, int> radiusSearch(
        const Eigen::Vector3d& search_Pt,
        double search_margin,
        double max_ray_length,
        const std::vector<NodePtr>& NodeList,
        const pcl::search::KdTree<pcl::PointXYZ>& kdtreeForRawMap,
        const std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>>& kdtreesForPolys
    ) 
    {
        double min_dis = radiusSearchOnRawMap(search_Pt, kdtreeForRawMap);
        int min_dis_node_index = -1;

        if (min_dis < search_margin) {
            return std::make_pair(min_dis, min_dis_node_index);
        }

        pcl::PointXYZ searchPoint;
        searchPoint.x = search_Pt(0);
        searchPoint.y = search_Pt(1);
        searchPoint.z = search_Pt(2);

        for (NodePtr node : NodeList) {
            if (node->rollbacked || node->isGate) continue;

            if (getDis(node->original_coord, search_Pt) > max_ray_length + min_dis) continue;

            std::vector<int> indices(1);
            std::vector<float> distance(1);

            kdtreesForPolys.at(node->index)
                ->nearestKSearch(searchPoint, 1, indices,
                                distance);
            double radius = sqrt(distance[0]);

            if (radius < min_dis) {
                min_dis = radius;
                min_dis_node_index = node->index;
            }
        }

        return std::make_pair(min_dis, min_dis_node_index);
    }

    double getDis(const NodePtr& node1, const NodePtr& node2) {
        return (node1->coord - node2->coord).norm();
    }

    double getDis(const Eigen::Vector3d& coord1, const Eigen::Vector3d& coord2) {
        return (coord1 - coord2).norm();
    }

    double getDis(const Eigen::Vector3d& coord, const NodePtr& node) {
        return (coord - node->coord).norm();
    }

    void centralizeNodePos(const NodePtr node) {
        int cnt = 0;
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();

        for (auto it = node->black_vertices.begin(); it != node->black_vertices.end();
            it++) {
            cnt++;
            sum = sum + (*it)->coord;
        }
        node->coord = sum / cnt;
    }

    double getNodeRadius(const NodePtr& node) {
        double node_radius = 0;
        for (VertexPtr bv : node->black_vertices) {
            node_radius += getDis(node->coord, bv->coord);
        }
        for (VertexPtr wv : node->white_vertices) {
            node_radius += getDis(node->coord, wv->coord);
        }
        node_radius = node_radius / (double)(node->black_vertices.size() +
                                             node->white_vertices.size());

        return node_radius;
    }

    double getVerticesRadius(const std::vector<Eigen::Vector3d>& vertices) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (Eigen::Vector3d v : vertices) {
            center += v;
        }
        center = center / (double)vertices.size();

        double radius = 0;
        for (Eigen::Vector3d v : vertices) {
            radius += getDis(center, v);
        }
        radius = radius / (double)vertices.size();

        return radius;
    }

    bool checkSegClear(
        const Eigen::Vector3d& pos1,
        const Eigen::Vector3d& pos2,
        const RaycastingContext& ray_context
    ) {
        double length = (pos2 - pos1).norm();
        double step_length = ray_context.map_context.resolution;

        Eigen::Vector3d step = step_length * (pos2 - pos1) / length;
        int num_steps = ceil(length / step_length);

        Eigen::Vector3d begin_pos = pos1;
        for (int i = 0; i < num_steps; i++) {
            Eigen::Vector3d check_pos = begin_pos + i * step;
            if (collisionCheck(check_pos, ray_context.search_margin, ray_context))
                return false;
        }
        if (collisionCheck(pos2, ray_context.search_margin, ray_context)) {
            return false;
        }
        return true;
    }

    bool collisionCheck(
        const Eigen::Vector3d& search_pt,
        double threshold,
        const RaycastingContext& ray_context
    ) {
        // Point cloud map
        if (ray_context.map_context.map_representation == 0) {
            double dis = radiusSearchOnRawMap(search_pt, ray_context.kdtreeForRawMap);
            if (dis < threshold)
                return true;
            else
                return false;
        }
        // Won't reach
        return true;
    }

    bool isSamePos(
        const Eigen::Vector3d& coord1,
        const Eigen::Vector3d& coord2,
        double threshold
    ) {
        return (getDis(coord1, coord2) < threshold);
    }
}