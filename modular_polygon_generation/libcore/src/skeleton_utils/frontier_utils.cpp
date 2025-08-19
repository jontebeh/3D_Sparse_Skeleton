#include <libcore/skeleton_utils/frontier_utils.hpp>
#include <libcore/skeleton_utils/facet_utils.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>
#include <libcore/skeleton_utils/raycasting.hpp>

namespace libcore
{
    bool checkPtOnFrontier(
        FrontierPtr ftr_ptr, 
        Eigen::Vector3d pt, 
        double search_margin
    ) {
        int num_facet = ftr_ptr->facets.size();
        for (int i = 0; i < num_facet; i++) {
            FacetPtr facet = ftr_ptr->facets.at(i);
            if (checkPtOnFacet(facet, pt, search_margin))
            return true;
            else
            continue;
        }
        return false;
    }

    FrontierPtr findFlowBackFrontier(
        Eigen::Vector3d pos, 
        int index,
        const std::vector<NodePtr>& center_NodeList,
        double max_ray_length,
        double search_margin
    ) {
        for (FrontierPtr f : center_NodeList.at(index)->frontiers) {
            if (getDis(pos, f->proj_center) > 2 * max_ray_length)
                continue;
            if (checkPtOnFrontier(f, pos, search_margin))
                return f;
        }

        return NULL;
    }

    void identifyFrontiers(
        NodePtr node,
        const Config& config,
        SharedVars& vars
    ) {
        std::vector<std::vector<VertexPtr>> bv_groups;
        int num_wv = node->white_vertices.size();
        for (int i = 0; i < num_wv; i++) {
            VertexPtr seed_wv = node->white_vertices.at(i);
            if (seed_wv->visited) continue;

            seed_wv->visited = true;
            std::vector<VertexPtr> group_bv;
            std::deque<VertexPtr> pending_wv;

            pending_wv.push_back(seed_wv);

            while (!pending_wv.empty()) {
                VertexPtr v = pending_wv.front();
                v->visited = true;
                pending_wv.pop_front();

                for (VertexPtr v_nbhd : v->connected_vertices) {
                    if (v_nbhd->type == VertexType::WHITE) {
                        if (v_nbhd->visited) continue;
                        Eigen::Vector3d midpt = (v->coord + v_nbhd->coord) / 2;
                        if (radiusSearch(
                            midpt, config.search_margin, config.max_ray_length,
                            vars.NodeList, vars.kdtreeForRawMap, vars.kdtreesForPolys
                            
                        ).first > 2 * config.search_margin) {
                            pending_wv.push_back(v_nbhd);
                        }
                    } else {
                        v_nbhd->critical = true;
                        group_bv.push_back(v_nbhd);
                    }
                }
            }

            if (group_bv.size() < 3) continue;
            for (VertexPtr v : group_bv) {
                v->visited = true;
            }
            bv_groups.push_back(group_bv);
        }

        // Filter black vertices
        int num_groups = bv_groups.size();
        for (int i = 0; i < num_groups; i++) {
            double mean_length = 0;
            double tolerance;
            for (VertexPtr v : bv_groups.at(i)) {
                mean_length += v->distance_to_center;
            }
            mean_length /= bv_groups.at(i).size();
            tolerance = mean_length * 0.3;
            int longest_index = -1;
            int shortest_index = -1;
            double longest = 0;
            double shortest = 9999;
            int num_ver = bv_groups.at(i).size();
            for (int j = 0; j < num_ver; j++) {
                if (onCeilOrFloor(
                        bv_groups.at(i).at(j)->coord,
                        config.z_min, 
                        config.z_max, 
                        config.search_margin
                    ) == 0)
                    continue;
                double dis = bv_groups.at(i).at(j)->distance_to_center;
                if (dis - mean_length > tolerance) {
                    if (dis > longest) {
                        longest_index = j;
                        longest = dis;
                    }
                } else if (mean_length - dis > tolerance) {
                    if (dis < shortest) {
                        shortest_index = j;
                        shortest = dis;
                    }
                }
            }
            if (longest_index != -1) {
                bv_groups.at(i).at(longest_index)->type = VertexType::GREY;
                bv_groups.at(i).at(longest_index)->critical = false;
            }
            if (shortest_index != -1) {
                bv_groups.at(i).at(shortest_index)->type = VertexType::GREY;
                bv_groups.at(i).at(shortest_index)->critical = false;
            }
        }

        // Inflate critical black vertices
        for (int i = 0; i < num_groups; i++) {
            for (VertexPtr v : bv_groups.at(i)) {
                for (VertexPtr v_nbhd : v->connected_vertices) {
                    if (v_nbhd->type == VertexType::BLACK)
                        v_nbhd->critical = true;
                }
            }
        }

        // Mesh1 is for only black polygon
        std::vector<quickhull::Vector3<double>> bv_for_mesh;
        for (VertexPtr bv : node->black_vertices) {
            if (onCeilOrFloor(bv->coord,
                config.z_min, 
                config.z_max, 
                config.search_margin
            ) != 0 && !bv->critical)
                continue;
            bv_for_mesh.push_back(
                node->sampling_directions.at(bv->sampling_direction_index)
            );
            bv->critical = true;
        }

        quickhull::QuickHull<double> qh;
        quickhull::HalfEdgeMesh<double, size_t> mesh1 =
            qh.getConvexHullAsMesh(&(bv_for_mesh)[0].x, (bv_for_mesh).size(), true);

        for (auto &face : mesh1.m_faces) {
            std::vector<VertexPtr> vertices;

            quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &halfedge =
                mesh1.m_halfEdges[face.m_halfEdgeIndex];

            for (int i = 0; i < 3; i++) {
            quickhull::Vector3<double> vertex_quickhull = mesh1.m_vertices[halfedge.m_endVertex];
            Eigen::Vector3d vertex_eigen;
            vertex_eigen << vertex_quickhull.x, 
                            vertex_quickhull.y,
                            vertex_quickhull.z;
            vertices.push_back(getVertexFromDire(node, vertex_eigen));

            halfedge = mesh1.m_halfEdges[halfedge.m_next];
            }

            FacetPtr new_facet = std::make_shared<Facet>(vertices, node);
            int ind = node->facets.size();
            node->facets.push_back(new_facet);
            new_facet->index = ind;
        }

        // Calculate outwards normal for each facet
        /* OLD CODE
        for (int i = 0; i < node->facets.size(); i++) {
            FacetPtr facet_ptr = node->facets.at(i);

            Eigen::Vector3d v1 = facet_ptr->vertices.at(1)->coord - facet_ptr->vertices.at(0)->coord;
            Eigen::Vector3d v2 = facet_ptr->vertices.at(2)->coord - facet_ptr->vertices.at(0)->coord;
            Eigen::Vector3d candidate_normal = v1.cross(v2);
            candidate_normal.normalize();

            Eigen::Vector3d pt_to_judge = facet_ptr->center + candidate_normal;
            if (checkPtInPolyhedron(node, pt_to_judge))
                facet_ptr->outwards_unit_normal = -candidate_normal;
            else
                facet_ptr->outwards_unit_normal = candidate_normal;
        }*/

        // Calculate outwards normal for each facet using Newell's method
        for (FacetPtr facet : node->facets) {
            Eigen::Vector3d normal = computeNormalNewell(facet->vertices);

            // if the point is inside flip it
            Eigen::Vector3d center_dir = facet->center - node->coord;
            if (center_dir.dot(normal) < 0) {
                facet->outwards_unit_normal = -normal;
            } else {
                facet->outwards_unit_normal = normal;
            }
            vars.vis.EnqueueDebugNode(facet->center, Eigen::Vector3d(0.0, 1.0, 0.0)); // TODO debugging
        }
        

        // Create frontiers given group black vertices
        for (int i = 0; i < num_groups; i++) {
            std::vector<FacetPtr> group_facets =
                findGroupFacetsFromVertices(
                    node, 
                    bv_groups.at(i),
                    config.z_min,
                    config.z_max,
                    config.search_margin
                );
            if (group_facets.empty()) continue;

            findNbhdFacets(group_facets);
            std::vector<std::vector<FacetPtr>> linked_groups;
            for (FacetPtr facet : group_facets) {
                if (facet->linked) continue;

                std::vector<FacetPtr> linked_group_facets;
                std::deque<FacetPtr> pending_facets;
                pending_facets.push_back(facet);
                while (!pending_facets.empty()) {
                    FacetPtr current_facet = pending_facets.front();
                    pending_facets.pop_front();
                    if (current_facet->linked) continue;
                    linked_group_facets.push_back(current_facet);
                    current_facet->linked = true;
                    for (FacetPtr nbhd : current_facet->nbhd_facets) {
                        if (!nbhd->linked)
                            pending_facets.push_back(nbhd);
                    }
                }

                linked_groups.push_back(linked_group_facets);
            }

            int num_linked_groups = linked_groups.size();
            for (int j = 0; j < num_linked_groups; j++) {
                std::vector<FrontierPtr> frontiers = 
                    splitFrontier(node, linked_groups.at(j), 
                                  config.max_facets_grouped,
                                  config.frontier_split_threshold);
                for (FrontierPtr f : frontiers) {
                    node->frontiers.push_back(f);
                }
            }
        }

        // Add bv_group for those connecting bvs having big diff in dis_to_center
        std::vector<FacetPtr> jump_facets;
        for (FacetPtr facet : node->facets) {
            if (facet->valid) continue;
            if (facetOnCeilOrFloor(
                facet,
                config.z_min,
                config.z_max,
                config.search_margin
            )) continue;

            int count_jump = 0;
            for (int i = 0; i < 3; i++) {
                VertexPtr v1 = facet->vertices.at(i);
                VertexPtr v2 = facet->vertices.at((i + 1) % 3);
                if (v1->distance_to_center > config.frontier_jump_threshold * v2->distance_to_center ||
                    v2->distance_to_center > config.frontier_jump_threshold * v1->distance_to_center) 
                {
                    count_jump++;
                }
            }
            if (count_jump > 1) {
            jump_facets.push_back(facet);
            facet->valid = true;
            }
        }

        findNbhdFacets(jump_facets);
        std::vector<std::vector<FacetPtr>> linked_groups;
        for (FacetPtr facet : jump_facets) {
            if (facet->linked) continue;

            std::vector<FacetPtr> linked_group_facets;
            std::deque<FacetPtr> pending_facets;
            pending_facets.push_back(facet);
            while (!pending_facets.empty()) {
                FacetPtr current_facet = pending_facets.front();
                pending_facets.pop_front();
                if (current_facet->linked) continue;
                linked_group_facets.push_back(current_facet);
                current_facet->linked = true;
                for (FacetPtr nbhd : current_facet->nbhd_facets) {
                    if (!nbhd->linked) pending_facets.push_back(nbhd);
                }
            }

            linked_groups.push_back(linked_group_facets);
        }

        int num_linked_groups = linked_groups.size();
        for (int j = 0; j < num_linked_groups; j++) {
            FrontierPtr new_fron = std::make_shared<Frontier>(linked_groups.at(j), node);
            if (initFrontier(new_fron)) node->frontiers.push_back(new_fron);
        }

        // Wish to expand frontier with more facets first
        sort(node->frontiers.begin(), node->frontiers.end(), compareFrontier);
        for (FrontierPtr f : node->frontiers) {
            if (f->facets.empty()) continue;
            vars.pending_frontiers.push_back(f);
            int ind = vars.loop_candidate_frontiers.size();
            f->index = ind;
            vars.loop_candidate_frontiers.push_back(f);
        }
    }

    std::vector<FrontierPtr> splitFrontier(
        NodePtr node, 
        std::vector<FacetPtr> group_facets,
        int max_facets_grouped,
        double frontier_split_threshold
    ) {
        std::vector<FrontierPtr> frontiers;
        if ((int)group_facets.size() <= max_facets_grouped) {
            Eigen::Vector3d avg_normal = Eigen::Vector3d::Zero();
            std::vector<FacetPtr> filtered_group_facets;
            for (FacetPtr f : group_facets) {
                avg_normal += f->outwards_unit_normal;
            }
            avg_normal.normalize();
            for (FacetPtr f : group_facets) {
                if (f->nbhd_facets.size() < 2) {
                    double angle = acos(avg_normal.dot(f->outwards_unit_normal));
                    if (angle > M_PI / 2.5) {
                        std::vector<FacetPtr> single_facet;
                        single_facet.push_back(f);
                        FrontierPtr new_frontier = std::make_shared<Frontier>(single_facet, node);
                        frontiers.push_back(new_frontier);
                        if (!initFrontier(new_frontier)) {
                            // Init frontier failed
                        }
                        continue;
                    }
                }
                filtered_group_facets.push_back(f);
            }
            FrontierPtr new_frontier = std::make_shared<Frontier>(filtered_group_facets, node);
            frontiers.push_back(new_frontier);
            if (!initFrontier(new_frontier)) {
                // init frontier failed
            }
        } else {
            for (FacetPtr facet : group_facets) {
                if (facet->visited)continue;

                facet->visited = true;
                Eigen::Vector3d normal = Eigen::Vector3d::Zero();
                std::vector<FacetPtr> small_group_facets;
                std::deque<FacetPtr> pending_facets;
                pending_facets.push_back(facet);

                while (!pending_facets.empty() &&
                        (int)small_group_facets.size() < max_facets_grouped) {
                    FacetPtr f = pending_facets.front();
                    pending_facets.pop_front();

                    if (!small_group_facets.empty()) {
                        if (acos(f->outwards_unit_normal.dot(normal)) <
                            M_PI / frontier_split_threshold) {
                            normal =
                                (normal * small_group_facets.size() + f->outwards_unit_normal) /
                                (small_group_facets.size() + 1);
                            normal.normalize();
                        } else continue;
                    } else {
                        normal = f->outwards_unit_normal;
                    }

                    f->visited = true;
                    small_group_facets.push_back(f);
                    for (FacetPtr f_nbhd : f->nbhd_facets) {
                        if (f_nbhd->visited) continue;
                        pending_facets.push_back(f_nbhd);
                    }
                }

                FrontierPtr new_frontier = std::make_shared<Frontier>(small_group_facets, node);
                frontiers.push_back(new_frontier);
                if (!initFrontier(new_frontier)) {
                    // init frontier failed
                }
            }
        }
        return frontiers;
    }

    bool initFrontier(FrontierPtr frontier) {
        // Set proj_center
        bool proj_center_found = false;

        // Line equation:
        // x = x0 + t * nx
        // y = y0 + t * ny
        // z = z0 + t * nz
        double x0 = frontier->avg_center(0);
        double y0 = frontier->avg_center(1);
        double z0 = frontier->avg_center(2);
        double nx = frontier->outwards_unit_normal(0);
        double ny = frontier->outwards_unit_normal(1);
        double nz = frontier->outwards_unit_normal(2);

        int num_facet = frontier->facets.size();
        if (num_facet == 0) {
            std::cerr << "No facets in frontier!" << std::endl;
            return false;
        }
        for (int i = 0; i < num_facet; i++) {
            double a = frontier->facets.at(i)->plane_equation(0);
            double b = frontier->facets.at(i)->plane_equation(1);
            double c = frontier->facets.at(i)->plane_equation(2);
            double d = frontier->facets.at(i)->plane_equation(3);

            double t = -(a * x0 + b * y0 + c * z0 + d) / (a * nx + b * ny + c * nz);

            Eigen::Vector3d intersection =
                frontier->avg_center + t * frontier->outwards_unit_normal;

            Eigen::Vector3d coord1 = frontier->facets.at(i)->vertices.at(0)->coord;
            Eigen::Vector3d coord2 = frontier->facets.at(i)->vertices.at(1)->coord;
            Eigen::Vector3d coord3 = frontier->facets.at(i)->vertices.at(2)->coord;

            Eigen::Vector3d cross1 = (coord2 - coord1).cross(intersection - coord1);
            Eigen::Vector3d cross2 = (coord3 - coord2).cross(intersection - coord2);
            Eigen::Vector3d cross3 = (coord1 - coord3).cross(intersection - coord3);
            if (cross1(0) * cross2(0) > 0 && cross2(0) * cross3(0) > 0 &&
                cross3(0) * cross1(0) > 0) {
                frontier->proj_center = intersection;
                frontier->proj_facet = frontier->facets.at(i);
                Eigen::Vector3d normal1 = frontier->outwards_unit_normal;
                Eigen::Vector3d normal2 = frontier->facets.at(i)->outwards_unit_normal;
                frontier->cos_theta =
                    normal1.dot(normal2) / (normal1.norm() * normal2.norm());
                proj_center_found = true;
                break;
            }
        }

        if (!proj_center_found) {
            double min_angle = M_PI;
            FacetPtr best_facet;
            for (FacetPtr f : frontier->facets) {
                double angle = acos(frontier->outwards_unit_normal.dot(f->outwards_unit_normal));
                if (angle < min_angle) {
                    min_angle = angle;
                    best_facet = f;
                }
            }
            frontier->proj_facet = best_facet;
            frontier->proj_center = best_facet->center;
        }

        // Set vertices
        for (FacetPtr facet : frontier->facets) {
            for (VertexPtr v_facet : facet->vertices) {
                bool exist = false;
                if (!frontier->vertices.empty()) {
                    for (VertexPtr v_frontier : frontier->vertices) {
                        if (isSamePos(v_facet->coord, v_frontier->coord)) {
                            exist = true;
                            break;
                        }
                    }
                }
                if (!exist)
                    frontier->vertices.push_back(v_facet);
            }
        }

        return proj_center_found;
    }

    bool compareFrontier(const FrontierPtr f1, const FrontierPtr f2) {
        return f1->facets.size() > f2->facets.size();
    }

    FrontierPtr pendingFrontiersPopFront(std::deque<FrontierPtr>& pending_frontiers) {
        FrontierPtr curFrontierPtr = pending_frontiers.front();
        pending_frontiers.pop_front();
        return curFrontierPtr;
    }

    void verifyFrontier(
        FrontierPtr ftr_ptr,
        const Config& config,
        SharedVars& vars
    ) {

        Eigen::Vector3d raycast_start_pt =
            ftr_ptr->proj_center + 2.0 * ftr_ptr->outwards_unit_normal * config.search_margin;

        std::pair<double, int> rs_result = radiusSearch(
            raycast_start_pt,
            config.search_margin,
            config.max_ray_length,
            vars.NodeList,
            vars.kdtreeForRawMap,
            vars.kdtreesForPolys
        );
        if (rs_result.first < config.search_margin) {
            ftr_ptr->valid = false;
            return;
        }

        std::pair<Eigen::Vector3d, int> raycast_result =
            raycast(
                raycast_start_pt, 
                ftr_ptr->outwards_unit_normal,
                config.max_expansion_ray_length,
                config, vars
            );
        Eigen::Vector3d hit_on_pcl = raycast_result.first;
        vars.vis.EnqueueDebugNode(hit_on_pcl, Eigen::Vector3d(1.0, 0.0, 1.0)); // TODO debugging magenta
        vars.vis.EnqueueDebugNode(ftr_ptr->proj_center, Eigen::Vector3d(1.0, 0.5, 1.0)); // TODO debugging purple
        vars.vis.EnqueueDebugNode(
            ftr_ptr->proj_center + ftr_ptr->outwards_unit_normal,
            Eigen::Vector3d(0.0, 1.0, 1.0)
        ); // TODO debugging blueish

        // raycast into a long corridor
        if (hit_on_pcl == raycast_start_pt) {
            Eigen::Vector3d new_node_candidate =
                ftr_ptr->proj_center +
                0.5 * config.max_expansion_ray_length * ftr_ptr->outwards_unit_normal;
            if (checkWithinBbx(new_node_candidate, 
                vars.bbx_min, 
                vars.bbx_max)) {
            ftr_ptr->next_node_pos = new_node_candidate;
            ftr_ptr->valid = true;
            } 
        }
        // normal case
        else if (getDis(hit_on_pcl, ftr_ptr->proj_center) >
                config.frontier_creation_threshold) {
            ftr_ptr->valid = true;
            ftr_ptr->next_node_pos = (hit_on_pcl + ftr_ptr->proj_center) / 2;
        }
    }
} // namespace libcore