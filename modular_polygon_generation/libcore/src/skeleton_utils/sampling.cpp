#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

#include <libcore/data_types/vertex.hpp>

#include <cmath>

namespace libcore
{
    void genSamplesOnUnitSphere(const Config& config, SharedVars& vars) {
        // Fibonicci sphere
        double phi = M_PI * (3 - sqrt(5));
        double x, y, z, radius, theta;

        for (int i = 0; i < config.sampling_density; i++) {
            y = 1 - 2 * ((float)i / (float)(config.sampling_density - 1));
            radius = sqrt(1 - y * y);
            theta = phi * i;
            x = cos(theta) * radius;
            z = sin(theta) * radius;

            Eigen::Vector3d sample;
            sample << x, y, z;
            vars.sample_directions.push_back(sample);
        }
    }

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const Config& config,
        SharedVars& vars) {
        for (auto it = vars.sample_directions.begin(); it != vars.sample_directions.end(); it++) {
            int index = nodePtr->sampling_directions.size();
            nodePtr->sampling_directions.push_back(quickhull::Vector3<double>((*it)(0), (*it)(1), (*it)(2)));

            std::pair<Eigen::Vector3d, int> raycast_result = raycast(
                nodePtr->coord, *it, 
                config.max_ray_length, 
                config, vars);
            Eigen::Vector3d newVertex = raycast_result.first;
            if (raycast_result.second == -2) {
                newVertex += (*it) * config.max_ray_length;
                VertexPtr new_white_vertex = std::make_shared<Vertex>(newVertex, (*it), VertexType::WHITE);
                new_white_vertex->sampling_direction_index = index;
                nodePtr->white_vertices.push_back(new_white_vertex);
            } else {
                VertexPtr new_black_vertex = std::make_shared<Vertex>(newVertex, (*it), VertexType::BLACK);
                new_black_vertex->collision_node_index = raycast_result.second;
                new_black_vertex->sampling_direction_index = index;
                new_black_vertex->distance_to_center = getDis(new_black_vertex->coord, nodePtr->coord);
                nodePtr->black_vertices.push_back(new_black_vertex);
            }
        }
    }

} // namespace libcore