#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

#include <libcore/data_types/vertex.hpp>

#include <cmath>

namespace libcore
{

    std::vector<Eigen::Vector3d> genUnitSphereDir(int num_samples) {
        // Fibonicci sphere
        double phi = M_PI * (3 - sqrt(5));
        double x, y, z, radius, theta;

        std::vector<Eigen::Vector3d> directions;
        directions.reserve(num_samples);

        for (int i = 0; i < num_samples; i++) {
            y = 1 - 2 * ((float)i / (float)(num_samples - 1));
            radius = sqrt(1 - y * y);
            theta = phi * i;
            x = cos(theta) * radius;
            z = sin(theta) * radius;

            Eigen::Vector3d sample;
            sample << x, y, z;
            sample.normalize();
            directions.push_back(sample);
        }
        return directions;
    }

    double superquadricDistanceAlongDirection(
        const Eigen::Vector3d& dir,
        double width, double height, double depth,
        double epsilon1, double epsilon2)
    {
        Eigen::Vector3d d = dir.normalized();

        double dx = d.x();
        double dy = d.y();
        double dz = d.z();

        double term_xy = std::pow(
            std::pow(std::abs(dx / width), 2.0 / epsilon2) +
            std::pow(std::abs(dy / depth), 2.0 / epsilon2),
            epsilon2 / epsilon1
        );

        double term_z = std::pow(std::abs(dz / height), 2.0 / epsilon1);

        double denom = term_xy + term_z;

        if (denom < 1e-12)
            return 0.0; // Avoid division by zero; direction aligned with null superquadric

        return std::pow(denom, -epsilon1 / 2.0);
    }


    void genSamplesOnShape(const Config& config, SharedVars& vars) {
        // Morphological shape sampling
        double width = config.sampling_width;
        double height = config.sampling_height;
        double depth = config.sampling_depth;
        double eps_1 = config.sampling_sharpness_eps1;
        double eps_2 = config.sampling_sharpness_eps2;
        int samples = config.sampling_density;

        vars.sample_directions.clear();
        vars.sample_directions.reserve(samples);

        std::vector<Eigen::Vector3d> unit_sphere_dirs = genUnitSphereDir(samples);

        for (const Eigen::Vector3d& dir : unit_sphere_dirs) {

            // Scale the direction vector by the superquadric function
            double scale = superquadricDistanceAlongDirection(dir, width, height, depth, eps_1, eps_2);
            if (scale > 0) {
                vars.sample_directions.emplace_back(dir, scale);
            }
        }
    }

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const Config& config,
        SharedVars& vars) 
    {
        // Generate sampling directions on the sample shape
        for (auto sample : vars.sample_directions) {
            Eigen::Vector3d direction = sample.first;
            double length = sample.second;
            // Add the sampling direction to the node's sampling directions
            int index = nodePtr->sampling_directions.size();
            nodePtr->sampling_directions.push_back(quickhull::Vector3<double>(direction(0), direction(1), direction(2)));

            std::pair<Eigen::Vector3d, int> raycast_result = raycast(
                nodePtr->coord, direction, 
                length,
                config, vars); // find the intersection point with the map

            Eigen::Vector3d newVertex = raycast_result.first;
            if (raycast_result.second == -2) { // if no point was found add a white vertex
                newVertex += direction * length;
                VertexPtr new_white_vertex = std::make_shared<Vertex>(newVertex, direction, VertexType::WHITE);
                new_white_vertex->sampling_direction_index = index;
                nodePtr->white_vertices.push_back(new_white_vertex);
            } else { // if a point was found add a black vertex
                VertexPtr new_black_vertex = std::make_shared<Vertex>(newVertex, direction, VertexType::BLACK);
                new_black_vertex->collision_node_index = raycast_result.second;
                new_black_vertex->sampling_direction_index = index;
                new_black_vertex->distance_to_center = getDis(new_black_vertex->coord, nodePtr->coord);
                nodePtr->black_vertices.push_back(new_black_vertex);
            }
        }
    }

} // namespace libcore