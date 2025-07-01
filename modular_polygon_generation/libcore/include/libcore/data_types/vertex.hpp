#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace libcore {

    /**
     * @brief The VertexType, either WHITE, BLACK, or GREY.
     */
    enum class VertexType { WHITE, BLACK, GREY };

    struct Vertex; // Forward declaration of Vertex
    /**
     * @brief Alias for a shared pointer to a Vertex.
     */
    using VertexPtr = std::shared_ptr<Vertex>;

    /**
     * @brief The Vertex structure represents a vertex in a 3D space with its coordinates,
     * unit direction, type, and connections to other vertices.
     */
    struct Vertex {
        // Coordinates of the vertex in 3D space
        Eigen::Vector3d coord;

        // Unit direction vector from this vertex
        Eigen::Vector3d unit_direction;

        // Type of the vertex (WHITE, BLACK, GREY)
        VertexType type;

        // List of vertices connected to this vertex
        std::vector<VertexPtr> connected_vertices;

        // Flags for vertex visitied state
        bool visited = false;

        // Flag indicating if this vertex is critical (part of a specific polygon)
        bool critical = false;

        // Index of the collision node associated with this vertex
        int collision_node_index;

        // Index of the sampling direction associated with this vertex
        int sampling_direction_index;

        // Distance to the center of the polygon or shape this vertex is part of
        double distance_to_center;

        // Default constructor
        Vertex() = default;

        /**
         * @brief Constructs a Vertex with specified coordinates, direction, and type.
         * @param coord_ Coordinates of the vertex.
         * @param dir_ Unit direction vector from this vertex.
         * @param t Type of the vertex (WHITE, BLACK, GREY).
         */
        Vertex(const Eigen::Vector3d& coord_, const Eigen::Vector3d& dir_, VertexType t)
            : coord(coord_), unit_direction(dir_), type(t) {}
    };
}  // namespace libcore