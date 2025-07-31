#include <libcore/skeleton_utils/facet_utils.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

#include <libcore/quickhull/QuickHull.hpp>

namespace libcore
{
    void identifyBwFacets(
        std::vector<std::pair<Eigen::Vector3d, double>> &sample_directions,
        std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions)
    {
        quickhull::QuickHull<double> qh;
        quickhull::HalfEdgeMesh<double, size_t> mesh2 = qh.getConvexHullAsMesh(
            &sample_directions[0].first(0), sample_directions.size(), true);

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
        const FacetPtr facet, 
        const Eigen::Vector3d pt,
        double search_margin
    ) {
        // Take any point on the plane
        Eigen::Vector3d origin = facet->vertices.at(0)->coord;
        Eigen::Vector3d origin_to_pt = pt - origin; // vector from origin to pt
        double dist_signed = origin_to_pt.dot(facet->outwards_unit_normal); // signed distance from the plane

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

    void identifyFacets(
        NodePtr node,
        const std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions
    )
    {
        for (std::vector<Eigen::Vector3d> facet_vertices : bw_facets_directions) {
            VertexPtr v1, v2, v3;
            v1 = getVertexFromDire(node, facet_vertices.at(0));
            v2 = getVertexFromDire(node, facet_vertices.at(1));
            v3 = getVertexFromDire(node, facet_vertices.at(2));
            v1->connected_vertices.push_back(v2);
            v2->connected_vertices.push_back(v3);
            v3->connected_vertices.push_back(v1);
        }
    }

    std::vector<FacetPtr> findGroupFacetsFromVertices(
        NodePtr node,
        std::vector<VertexPtr> group_bv,
        const double z_min,
        const double z_max,
        const double search_margin
    ) {
        std::vector<FacetPtr> group_facets;
        for (FacetPtr f : node->facets) {
            bool good = true;
            for (VertexPtr v_facet : f->vertices) {
                bool notIncluded = true;
                for (VertexPtr v_group : group_bv) {
                    if (!v_group->critical) continue;
                    if (isSamePos(v_facet->coord, v_group->coord)) {
                        notIncluded = false;
                        break;
                    }
                }
                if (notIncluded) {
                    good = false;
                    break;
                }
            }
            // All of the three vertices are included in the group_bv
            if (good) {
                // Exclude facets completely on ceil or floor to improve frontier accuracy
                if ((
                    onCeilOrFloor(
                        f->vertices.at(0)->coord,
                        z_min, z_max, search_margin
                    ) == 1 &&
                    onCeilOrFloor(
                        f->vertices.at(1)->coord,
                        z_min, z_max, search_margin
                    ) == 1 &&
                    onCeilOrFloor(
                        f->vertices.at(2)->coord,
                        z_min, z_max, search_margin
                    ) == 1) ||
                    (
                    onCeilOrFloor(
                        f->vertices.at(0)->coord,
                        z_min, z_max, search_margin
                    ) == -1 &&
                    onCeilOrFloor(
                        f->vertices.at(1)->coord,
                        z_min, z_max, search_margin
                    ) == -1 &&
                    onCeilOrFloor(
                        f->vertices.at(2)->coord,
                        z_min, z_max, search_margin
                    ) == -1)) continue;
                group_facets.push_back(f);
                f->valid = true;
            }
        }
        return group_facets;
    }

    void findNbhdFacets(std::vector<FacetPtr>& facets) {
        int num_facet = facets.size();
        for (int i = 0; i < num_facet - 1; i++) {
            for (int j = i + 1; j < num_facet; j++) {
                FacetPtr f1 = facets.at(i);
                FacetPtr f2 = facets.at(j);

                if (f1->nbhd_facets.size() == 3) break;

                int same_vertices_count = 0;
                for (int s = 0; s < 3; s++) {
                    for (int t = 0; t < 3; t++) {
                        if (isSamePos(f1->vertices.at(s)->coord, 
                                      f2->vertices.at(t)->coord)) {
                            same_vertices_count++;
                            break;
                        }
                    }
                }
                if (same_vertices_count == 2) {
                    f1->nbhd_facets.push_back(f2);
                    f2->nbhd_facets.push_back(f1);
                }
            }
        }
    }

    bool facetOnCeilOrFloor(
        const FacetPtr f,
        const double z_min,
        const double z_max,
        const double search_margin
    ) {
        return (
            onCeilOrFloor(
                f->vertices.at(0)->coord,
                z_min, z_max, search_margin
            ) == -1 &&
            onCeilOrFloor(
                f->vertices.at(1)->coord,
                z_min, z_max, search_margin
            ) == -1 &&
            onCeilOrFloor(
                f->vertices.at(2)->coord,
                z_min, z_max, search_margin
            ) == -1) || (
            onCeilOrFloor(
                f->vertices.at(0)->coord,
                z_min, z_max, search_margin
            ) == 1 &&
            onCeilOrFloor(
                f->vertices.at(1)->coord,
                z_min, z_max, search_margin
            ) == 1 &&
            onCeilOrFloor(
                f->vertices.at(2)->coord,
                z_min, z_max, search_margin
            ) == 1
        );
    }

    void addFacetsToPcl(
        NodePtr nodePtr,
        const double resolution,
        std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>>& kdtreesForPolys
    ) {
        pcl::PointCloud<pcl::PointXYZ> poly_pcl;
        int num_facet = nodePtr->facets.size();
        for (int i = 0; i < num_facet; i++) {
            FacetPtr facet = nodePtr->facets.at(i);

            std::vector<Eigen::Vector3d> start_list;
            std::vector<double> length_list;

            Eigen::Vector3d top_vertex = facet->vertices.at(0)->coord;
            Eigen::Vector3d left_vertex = facet->vertices.at(1)->coord;
            Eigen::Vector3d right_vertex = facet->vertices.at(2)->coord;

            Eigen::Vector3d left_to_top = top_vertex - left_vertex;
            Eigen::Vector3d left_to_top_unit = left_to_top / left_to_top.norm();
            Eigen::Vector3d left_to_right = right_vertex - left_vertex;
            Eigen::Vector3d right_to_top = top_vertex - right_vertex;
            Eigen::Vector3d right_to_top_unit = right_to_top / right_to_top.norm();
            Eigen::Vector3d right_to_left = left_vertex - right_vertex;

            double theta = acos(left_to_top.dot(left_to_right) /
                               (left_to_top.norm() * left_to_right.norm()));
            double theta_right = acos(right_to_top.dot(right_to_left) /
                                     (right_to_top.norm() * right_to_left.norm()));
            double step_length = resolution / 2;
            double start_pt_step = step_length / sin(theta);
            double end_pt_step = step_length / sin(theta_right);

            int num_start_pt = ceil(left_to_top.norm() / start_pt_step);
            double length;
            for (int j = 0; j < num_start_pt; j++) {
                Eigen::Vector3d start =
                    left_vertex + j * start_pt_step * left_to_top_unit;
                Eigen::Vector3d end = 
                    right_vertex + j * end_pt_step * right_to_top_unit;
                start_list.push_back(start);
                length = (end - start).norm();
                length_list.push_back(length);
            }

            Eigen::Vector3d direction_unit = right_vertex - left_vertex;
            direction_unit.normalize();
            for (int j = 0; j < num_start_pt; j++) {
                int num_pts = ceil(length_list.at(j) / step_length);
                for (int k = 0; k < num_pts; k++) {
                    Eigen::Vector3d pt_to_push =
                        start_list.at(j) + k * direction_unit * step_length;
                    for (int s = 0; s < 4; s++) {
                        poly_pcl.points.push_back(
                            pcl::PointXYZ(pt_to_push(0), pt_to_push(1), pt_to_push(2)));
                        pt_to_push += (-facet->outwards_unit_normal) * step_length;
                    }
                }
                Eigen::Vector3d end_to_push =
                    start_list.at(j) + length_list.at(j) * direction_unit;
                for (int s = 0; s < 4; s++) {
                    poly_pcl.points.push_back(
                        pcl::PointXYZ(end_to_push(0), end_to_push(1), end_to_push(2)));
                    end_to_push += (-facet->outwards_unit_normal) * step_length;
                }
            }
            poly_pcl.points.push_back(
                pcl::PointXYZ(top_vertex(0), top_vertex(1), top_vertex(2)));
        }

        std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> new_poly(
            new pcl::search::KdTree<pcl::PointXYZ>);
        new_poly->setInputCloud(poly_pcl.makeShared());
        kdtreesForPolys.push_back(new_poly);
    }
} // namespace libcore