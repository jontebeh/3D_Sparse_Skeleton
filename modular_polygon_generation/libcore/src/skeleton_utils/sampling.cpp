#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

#include <libcore/data_types/vertex.hpp>

#include <cmath>

namespace libcore
{
    void genSamplesOnUnitSphere(const SamplingContext& c)
    {
        // Fibonicci sphere
        double phi = M_PI * (3 - sqrt(5));
        double x, y, z, radius, theta;

        for (int i = 0; i < c.sampling_density; i++) {
            y = 1 - 2 * ((float)i / (float)(c.sampling_density - 1));
            radius = sqrt(1 - y * y);
            theta = phi * i;
            x = cos(theta) * radius;
            z = sin(theta) * radius;

            Eigen::Vector3d sample;
            sample << x, y, z;
            c.sample_directions.push_back(sample);
        }
    }

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const SamplingContext& c
    ) {
        for (auto it = c.sample_directions.begin(); it != c.sample_directions.end(); it++) {
            int index = nodePtr->sampling_directions.size();
            nodePtr->sampling_directions.push_back(quickhull::Vector3<double>((*it)(0), (*it)(1), (*it)(2)));

            std::pair<Eigen::Vector3d, int> raycast_result = raycast(nodePtr->coord, *it, c.ray_context.max_ray_length, c.ray_context);
            Eigen::Vector3d newVertex = raycast_result.first;
            if (raycast_result.second == -2) {
                newVertex += (*it) * c.ray_context.max_ray_length;
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