#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <libcore/data_types/vertex.hpp>

namespace libcore {

    struct Node;
    using NodePtr = std::shared_ptr<Node>;

    struct Facet;
    /**
     * @brief Alias for a shared pointer to a Facet.
     */
    using FacetPtr = std::shared_ptr<Facet>;

    /**
     * @brief The Facet structure represents a polygonal facet in 3D space.
     * It contains vertices, normal vector, center, and plane equation.
     */
    struct Facet {
        // Index of the facet in the mesh
        int index = -1;

        // Vertices of the facet
        std::vector<VertexPtr> vertices;

        // Unit normal vector pointing outwards from the facet
        Eigen::Vector3d outwards_unit_normal;

        // Center of the facet, calculated as the average of its vertices' coordinates
        Eigen::Vector3d center;

        // Position of the seed node associated with this facet
        Eigen::Vector3d seed_node_pos;

        // Plane equation of the facet in the form ax + by + cz + d = 0
        Eigen::Vector4d plane_equation;

        // Pointer to the master node associated with this facet
        NodePtr master_node;

        // Neighborhood facets that are adjacent to this facet
        std::vector<FacetPtr> nbhd_facets;

        // Neighborhood facets that are invalid (not part of the mesh)
        std::vector<FacetPtr> nbhd_invalid_facets;


        // Flags for facet state
        bool valid = false;

        // Indicates if the facet has been visited during processing
        bool visited = false;

        // Indicates if the facet is linked to other facets or nodes
        bool linked = false;

        // Default constructor
        Facet() = default;

        /**
         * @brief Constructs a Facet with specified vertices and master node.
         * @param verts Vertices of the facet.
         * @param node Pointer to the master node associated with this facet.
         */
        Facet(const std::vector<VertexPtr>& verts, const NodePtr& node) : 
            vertices(verts), 
            master_node(node) 
        {
            // Ensure there are exactly three vertices for a triangular facet
            center = (verts[0]->coord + verts[1]->coord + verts[2]->coord) / 3.0;

            // Compute the plane equation: ax + by + cz + d = 0
            Eigen::Vector3d v1 = verts[1]->coord - verts[0]->coord;
            Eigen::Vector3d v2 = verts[2]->coord - verts[0]->coord;
            Eigen::Vector3d normal = v1.cross(v2).normalized();

            // Set the outwards unit normal vector
            outwards_unit_normal = normal;

            // Calculate the plane equation coefficients
            double d = -normal.dot(Eigen::Vector3d(verts[0]->coord));

            // Store the plane equation in the form [a, b, c, d]
            plane_equation << normal, d;
        }
    };

}  // namespace libcore