#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <libcore/data_types/frontier.hpp>
#include <libcore/data_types/vertex.hpp>
#include <libcore/data_types/facet.hpp>

#include <libcore/quickhull/QuickHull.hpp>

namespace libcore {

    struct Node;
    /**
     * @brief Alias for a shared pointer to a Node.
     */
    using NodePtr = std::shared_ptr<Node>;

    /**
     * @brief The Node structure represents a node in the 3D mesh.
     * It contains coordinates, seed frontier, clearance, distance to floor,
     * and associated vertices, facets, and frontiers.
     */
    struct Node {
        // Index of the node in the mesh
        int index;

        // Coordinates of the node in 3D space
        Eigen::Vector3d coord;

        // Coordinates of the original position of the node before any transformations
        Eigen::Vector3d original_coord;

        // Pointer to the seed frontier from which this node was generated
        FrontierPtr seed_frontier;

        // Unit direction vector from this node
        double clearance;

        // Distance to the floor (ground plane)
        double dis_to_floor;

        // Flag indicating if this node is a gate node
        bool isGate = false;

        // Flag indicating if this node is a master node
        bool rollbacked = false;

        // Black vertices associated with this node
        std::vector<VertexPtr> black_vertices;

        // White vertices associated with this node
        std::vector<VertexPtr> white_vertices;

        // Facets associated with this node
        std::vector<FacetPtr> facets;

        // Frontiers associated with this node
        std::vector<FrontierPtr> frontiers;

        // List of nodes connected to this node
        std::vector<NodePtr> connected_nodes;

        // List of nodes that are part of the same polygon as this node
        std::vector<quickhull::Vector3<double>> sampling_directions;

        // List of valid sampling directions for this node
        std::vector<quickhull::Vector3<double>> valid_sampling_directions;

        // Default constructor
        Node() = default;

        /**
         * @brief Constructs a Node with specified coordinates and seed frontier.
         * @param coord_ Coordinates of the node.
         * @param seed Pointer to the seed frontier associated with this node.
         * @param gate Flag indicating if this node is a gate node (default is false).
         */
        Node(const Eigen::Vector3d& coord_, FrontierPtr seed, bool gate = false)
            : coord(coord_), original_coord(coord_), seed_frontier(seed), isGate(gate) {}
    };

}  // namespace libcore