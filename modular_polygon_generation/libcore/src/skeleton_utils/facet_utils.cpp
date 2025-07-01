#include <libcore/skeleton_utils/facet_utils.hpp>

namespace libcore
{
    void identifyBwFacets(std::vector<Eigen::Vector3d> &sample_directions,
                          std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions)
    {
        // Mesh2 is for black and white polygon
        quickhull::QuickHull<double> qh;
        quickhull::HalfEdgeMesh<double, size_t> mesh2 = qh.getConvexHullAsMesh(
            &sample_directions[0](0), sample_directions.size(), true);

        for (auto &face : mesh2.m_faces) {
            quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &halfedge =
                mesh2.m_halfEdges[face.m_halfEdgeIndex];

            quickhull::Vector3<double> vertex_quickhull;
            std::vector<Eigen::Vector3d> vertices_eigen;
            Eigen::Vector3d vertex_eigen;
            for (int i = 0; i < 3; i++) {
                vertex_quickhull = mesh2.m_vertices[halfedge.m_endVertex];
                vertex_eigen = Eigen::Vector3d::Zero();
                vertex_eigen << vertex_quickhull.x, 
                                vertex_quickhull.y,
                                vertex_quickhull.z;
                vertices_eigen.push_back(vertex_eigen);
                halfedge = mesh2.m_halfEdges[halfedge.m_next];
            }

            bw_facets_directions.push_back(vertices_eigen);
        }
    }

    bool checkPtOnFacet(
        FacetPtr facet, 
        Eigen::Vector3d pt,
        double search_margin
    ) {
        // Take any point on the plane
        Eigen::Vector3d origin = facet->vertices.at(0)->coord;
        Eigen::Vector3d origin_to_pt = pt - origin;
        double dist_signed = origin_to_pt.dot(facet->outwards_unit_normal);

        if (fabs(dist_signed) >= 2 * search_margin)
            return false;

        Eigen::Vector3d projected_pt = pt - dist_signed * facet->outwards_unit_normal;
        Eigen::Vector3d coord1 = facet->vertices.at(0)->coord;
        Eigen::Vector3d coord2 = facet->vertices.at(1)->coord;
        Eigen::Vector3d coord3 = facet->vertices.at(2)->coord;
        Eigen::Vector3d cross1 = (coord2 - coord1).cross(projected_pt - coord1);
        Eigen::Vector3d cross2 = (coord3 - coord2).cross(projected_pt - coord2);
        Eigen::Vector3d cross3 = (coord1 - coord3).cross(projected_pt - coord3);

        if (cross1(0) * cross2(0) > 0 && cross2(0) * cross3(0) > 0 &&
            cross3(0) * cross1(0) > 0)
            return true;
        else
            return false;
    }
} // namespace libcore