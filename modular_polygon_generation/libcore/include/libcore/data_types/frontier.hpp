#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <libcore/data_types/facet.hpp>

namespace libcore {

    struct Frontier;
    /**
     * @brief Alias for a shared pointer to a Frontier.
     */
    using FrontierPtr = std::shared_ptr<Frontier>;

    /**
     * @brief The Frontier structure represents a collection of facets and vertices
     * that form a frontier in the 3D mesh.
     */
    struct Frontier {
        // Index of the frontier in the mesh
        int index = -1;

        // List of facets that belong to this frontier
        std::vector<FacetPtr> facets;

        // List of vertices that belong to this frontier
        std::vector<VertexPtr> vertices;

        // Average center of the facets in this frontier
        Eigen::Vector3d avg_center;

        // Outwards unit normal vector of the facets in this frontier
        Eigen::Vector3d outwards_unit_normal;

        // Projected center of the frontier, used for visualization or processing
        Eigen::Vector3d proj_center;

        // Projected facet for visualization or processing
        FacetPtr proj_facet;

        // Position of the next node in the frontier
        Eigen::Vector3d next_node_pos;

        // Flags for frontier state
        bool valid = false;
        bool deleted = false;

        // Pointer to the master node associated with this frontier
        NodePtr master_node;

        // Pointer to the gate node, which is a special node in the frontier
        NodePtr gate_node = nullptr;

        // Default constructor
        Frontier() = default;

        /**
         * @brief Constructs a Frontier from a list of facets and a master node.
         * @param facet_list List of facets that belong to this frontier.
         * @param node Pointer to the master node associated with this frontier.
         */
        Frontier(const std::vector<FacetPtr>& facet_list, const NodePtr& node) : 
            facets(facet_list), 
            master_node(node) 
        {
            // Calculate the average center and outwards unit normal of the facets
            Eigen::Vector3d coord_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();

            // Iterate through the facets to compute the average center and normal
            for (const auto& f : facet_list) {
                coord_sum += f->center;
                normal_sum += f->outwards_unit_normal;
            }

            // Normalize the outwards unit normal vector
            int num_facet = static_cast<int>(facet_list.size());
            avg_center = coord_sum / num_facet;
            outwards_unit_normal = normal_sum / num_facet;
        }
    };

}  // namespace libcore