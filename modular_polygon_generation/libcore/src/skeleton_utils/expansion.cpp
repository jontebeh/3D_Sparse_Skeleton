#include <libcore/skeleton_utils/expansion.hpp>
#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/facet_utils.hpp>
#include <libcore/skeleton_utils/frontier_utils.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

namespace libcore
{
    void skeletonExpansion(const ExpansionContext& c) {
        Eigen::Vector3d startPt = c.startPt;

        std::vector<Eigen::Vector3d> sample_directions;
        genSamplesOnUnitSphere(c.sampling_context);

        std::vector<std::vector<Eigen::Vector3d>> bw_facets_directions;
        identifyBwFacets(sample_directions, bw_facets_directions);

        setStartPt(startPt, c);

        FrontierPtr cur_frontier;
        while (!pending_frontiers.empty()) {
            ros::Time begin = ros::Time::now();

            cur_frontier = pendingFrontiersPopFront();
            if (cur_frontier->deleted) continue;
            if (cur_frontier == NULL ) continue;

            verifyFrontier(cur_frontier);

            if (!cur_frontier->valid) {
                Eigen::Vector3d prev_proj_center = cur_frontier->proj_center;
                Eigen::Vector3d prev_normal      = cur_frontier->outwards_unit_normal;

                cur_frontier->proj_center = cur_frontier->proj_facet->center;

                verifyFrontier(cur_frontier);

                if (!cur_frontier->valid) {
                    cur_frontier->proj_center          = prev_proj_center;
                    cur_frontier->outwards_unit_normal = cur_frontier->proj_facet->outwards_unit_normal;
                    verifyFrontier(cur_frontier);

                    if (!cur_frontier->valid) {
                        cur_frontier->proj_center = cur_frontier->proj_facet->center;
                        verifyFrontier(cur_frontier);

                        if (!cur_frontier->valid) {
                            cur_frontier->outwards_unit_normal = prev_normal;
                            for (FacetPtr candidate_facet : cur_frontier->proj_facet->nbhd_facets) {
                                cur_frontier->proj_center = candidate_facet->center;
                                verifyFrontier(cur_frontier);
                                if (!cur_frontier->valid) {
                                    cur_frontier->outwards_unit_normal = candidate_facet->outwards_unit_normal;
                                    verifyFrontier(cur_frontier);
                                    if (cur_frontier->valid) break;
                                } else break;
                            }
                        }
                    }
                }
            }
            ros::Time finish = ros::Time::now();
            verifyFrontier_timing += (finish - begin).toSec();
            begin = ros::Time::now();
            if (cur_frontier->valid) {
                if (!processFrontier(cur_frontier)) {
                    // ROS_INFO("processFrontier fails.");
                } else {
                    if (!_visualize_final_result_only) visualization();
                    if (_debug_mode) {
                        ROS_WARN("Expanded a valid frontier.");
                        visualization();
                        getchar();
                    }
                }
            } else {
                if (_debug_mode) {
                    ROS_WARN("Frontier not valid!");
                }
            }
            finish = ros::Time::now();
            processFrontier_timing += (finish - begin).toSec();
        }
    }

    void setStartPt(Eigen::Vector3d& startPt, const ExpansionContext& c) {
        NodePtr start_node = std::make_shared<Node>(startPt, nullptr);
        initNode(start_node, c);
    }

    bool initNode(NodePtr curNodePtr, const ExpansionContext& c) {
        if (!checkWithinBbx(curNodePtr->coord,
                            c.bbx_min,
                            c.bbx_max)) return false;
        
        double max_ray_length = c.ray_context.max_ray_length;
        double max_height_diff = c.max_height_diff;
        if (!checkFloor(curNodePtr, max_ray_length, max_height_diff, c.ray_context)) return false;

        if (!curNodePtr->isGate) {
            genBlackAndWhiteVertices(
                curNodePtr,
                c.sampling_context
            );

            if (curNodePtr->black_vertices.size() < 4) return false;

            centralizeNodePos(curNodePtr);

            double radius = getNodeRadius(curNodePtr);
            if (radius < c.min_node_radius && 
                curNodePtr->white_vertices.empty()) return false;

            // Absorb node inside node
            if (curNodePtr->seed_frontier != NULL) {
                bool diff_ind = false;
                int ind = curNodePtr->black_vertices.at(0)->collision_node_index;
                for (VertexPtr v : curNodePtr->black_vertices) {
                    if (v->collision_node_index != ind) {
                        diff_ind = true;
                        break;
                    }
                }
                if (!diff_ind) return false;
            }
            findFlowBack(curNodePtr, c);

            identifyFacets(curNodePtr);

            identifyFrontiers(curNodePtr);

            addFacetsToPcl(curNodePtr);
        }

        recordNode(curNodePtr, c);
        return true;
    }

    void findFlowBack(NodePtr node, const ExpansionContext& c) {
        if (node->seed_frontier == NULL) return;

        int size = c.loop_candidate_frontiers.size();
        std::vector<std::vector<Eigen::Vector3d>> flow_back_frontier_log(size);
        std::vector<FrontierPtr> frontiers(size);
        std::vector<int> pending_frontier_index;
        std::vector<int> collision_node_index_log;

        // Count number of contact black vertices on each frontiers
        for (VertexPtr v : node->black_vertices) {
            // only process vertices collide with other polyhedrons
            if (v->collision_node_index < 0) continue;
            // hit_on_pcl is on seed_frontier which is already connected
            if (checkPtOnFrontier(
                node->seed_frontier, 
                v->coord,
                c.ray_context.search_margin
            )) continue;

            FrontierPtr loop_ftr = findFlowBackFrontier(
                v->coord, 
                v->collision_node_index,
                c.center_NodeList,
                c.ray_context.max_ray_length,
                c.ray_context.search_margin
            );
            if (loop_ftr == NULL) {
                collision_node_index_log.push_back(v->collision_node_index);
                continue;
            }

            flow_back_frontier_log.at(loop_ftr->index).push_back(v->coord);
            frontiers.at(loop_ftr->index) = loop_ftr;
        }

        // Sort decreasingly: flowback frontiers with more hits first
        for (int i = 0; i < size; i++) {
            if (flow_back_frontier_log.at(i).empty()) continue;
            if (((int)flow_back_frontier_log.at(i).size() < c.min_flowback_creation_threshold) && 
                (getVerticesRadius(flow_back_frontier_log.at(i)) < c.min_flowback_creation_radius_threshold)
            ) continue;

            if (pending_frontier_index.empty()) pending_frontier_index.push_back(i);
            else {
                for (auto it = pending_frontier_index.begin();
                    it != pending_frontier_index.end(); it++) 
                {
                    if (flow_back_frontier_log.at(*it).size() <
                        flow_back_frontier_log.at(i).size())
                    {
                        pending_frontier_index.insert(it, i);
                        break;
                    }
                    pending_frontier_index.push_back(i);
                    break;
                }
            }
        }

        // Start flowback
        int size_pending_frontier = pending_frontier_index.size();

        std::vector<Eigen::Vector3d> connected_node_pos;
        if (node->seed_frontier->gate_node->connected_nodes.empty()) {
            connected_node_pos.push_back(node->seed_frontier->master_node->coord);
        } else {
            for (NodePtr gate_con_node :
                node->seed_frontier->gate_node->connected_nodes) 
            {
                connected_node_pos.push_back(gate_con_node->coord);
            }
        }
        for (int i = 0; i < size_pending_frontier; i++) {
            int ind = pending_frontier_index.at(i);
            FrontierPtr flowback_frontier = frontiers.at(ind);

            // Unsafe loop: loop seg is not obstacle-free
            Eigen::Vector3d end_pt_on_frontier;
            if (flowback_frontier->gate_node == NULL) end_pt_on_frontier = flowback_frontier->proj_center;
            else end_pt_on_frontier = flowback_frontier->gate_node->coord;
            if (!checkSegClear(node->coord, end_pt_on_frontier) ||
                !checkSegClear(flowback_frontier->master_node->coord, end_pt_on_frontier)
            ) continue;

            // Bad loop: loop only contains 4 nodes
            if (c.bad_loop_setting) {
                bool bad_loop = false;
                for (Eigen::Vector3d pos : connected_node_pos) {
                    if (flowback_frontier->gate_node == NULL ||
                        flowback_frontier->gate_node->rollbacked) {
                        if (isSamePos(pos, flowback_frontier->master_node->coord)) bad_loop = true;
                    } else {
                        for (NodePtr frontier_con_node : flowback_frontier->gate_node->connected_nodes) {
                            if (isSamePos(pos, frontier_con_node->coord)) {
                                bad_loop = true;
                                break;
                            }
                        }
                    }
                    if (bad_loop)
                    break;
                }
                if (bad_loop) continue;
            }

            // Create flowback: new gate node if necessary
            if (flowback_frontier->gate_node == NULL) {
                NodePtr new_gate = std::make_shared<Node>(flowback_frontier->proj_center, flowback_frontier, true);
                if (initNode(new_gate, c)) {
                    flowback_frontier->gate_node = new_gate;
                    flowback_frontier->master_node->connected_nodes.push_back(new_gate);
                    new_gate->connected_nodes.push_back(flowback_frontier->master_node);
                    node->connected_nodes.push_back(new_gate);
                    new_gate->connected_nodes.push_back(node);
                } else continue;
            } else if (flowback_frontier->gate_node->rollbacked) {
                flowback_frontier->gate_node->rollbacked = false;
                recordNode(flowback_frontier->gate_node);
                flowback_frontier->master_node->connected_nodes.push_back(
                    flowback_frontier->gate_node);
                flowback_frontier->gate_node->connected_nodes.push_back(
                    flowback_frontier->master_node);
                node->connected_nodes.push_back(flowback_frontier->gate_node);
                flowback_frontier->gate_node->connected_nodes.push_back(node);
            } else {
                node->connected_nodes.push_back(flowback_frontier->gate_node);
                flowback_frontier->gate_node->connected_nodes.push_back(node);
            }

            for (NodePtr frontier_con_node :
                flowback_frontier->gate_node->connected_nodes) {
                connected_node_pos.push_back(frontier_con_node->coord);
            }
        }
    }

    void recordNode(NodePtr new_node, const ExpansionContext& c) {
        c.ray_context.NodeList.push_back(new_node);
        c.nodes_pcl.points.push_back(pcl::PointXYZ(
            new_node->coord(0), new_node->coord(1), new_node->coord(2))
        );

        if (!new_node->isGate) {
            int index = c.center_NodeList.size();
            new_node->index = index;
            c.center_NodeList.push_back(new_node);
        }
    }

} // namespace libcore
