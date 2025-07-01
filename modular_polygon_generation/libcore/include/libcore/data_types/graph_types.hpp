#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace libcore {

    /**
     * @brief Represents an edge between two nodes in the skeleton graph.
     */
    struct Edge {
        int from;  // Index of the start node
        int to;    // Index of the end node

        /**
         * @brief Constructs an Edge with specified from and to indices.
         * @param f Index of the start node
         * @param t Index of the end node
         */
        Edge(int f, int t) : from(f), to(t) {}
    };

    /**
     * @brief Represents a complete skeleton output as a graph.
     */
    struct Skeleton {
        std::vector<Eigen::Vector3d> nodes;  // Coordinates of all nodes
        std::vector<Edge> edges;             // Edges between nodes (by index)
    };

}  // namespace libcore