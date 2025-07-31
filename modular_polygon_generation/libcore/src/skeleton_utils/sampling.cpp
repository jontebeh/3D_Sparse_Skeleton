#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

#include <libcore/data_types/vertex.hpp>

#include <cmath>

namespace libcore
{

    double superquadricFunction(
        const Eigen::Vector3d& dir,
        double width, double height, double depth, double sharpness)
    {
        double x = dir.x();
        double y = dir.y();
        double z = dir.z();

        double term1 = std::pow(
            std::pow(std::abs(x / width), 2.0 / sharpness) +
            std::pow(std::abs(y / height), 2.0 / sharpness),
            sharpness / 2.0
        );
        double term2 = std::pow(std::abs(z / depth), 2.0 / sharpness);

        double denom = term1 + term2;
        if (denom < 1e-6) denom = 1e-6;  // avoid division by zero

        return 1.0 / std::pow(denom, 0.5);
    }

    void genSamplesOnShape(const Config& config, SharedVars& vars) {
        // Morphological shape sampling
        double width = config.sampling_width;
        double height = config.sampling_height;
        double depth = config.sampling_depth;
        double sharpness = config.sampling_sharpness;
        int samples = config.sampling_density;

        // Generate sampling directions based on the shape parameters
        int lat_samples = std::sqrt(samples);
        int lon_samples = lat_samples; // Assuming a square distribution for simplicity

        vars.sample_directions.clear();
        //vars.sample_directions.reserve(lat_samples * lon_samples);

        for (int i = 0; i < lat_samples; ++i) {
            double theta = M_PI * double(i) / (lat_samples - 1); // Polar angle
            for (int j = 0; j < lon_samples; ++j) {
                double phi = 2 * M_PI * double(j) / lon_samples; // Azimuthal angle

                // Spherical to Cartesian conversion
                Eigen::Vector3d direction(
                    std::sin(theta) * std::cos(phi),
                    std::sin(theta) * std::sin(phi),
                    std::cos(theta)
                );
                direction.normalize();

                // Scale the direction vector by the superquadric function
                double scale = superquadricFunction(direction, width, height, depth, sharpness);
                if (scale > 0) {
                    direction.normalize(); // Normalize the direction vector
                    vars.sample_directions.emplace_back(direction, scale);
                }
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
                config.max_ray_length, 
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