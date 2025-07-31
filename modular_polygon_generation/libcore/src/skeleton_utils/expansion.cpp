#include <libcore/skeleton_utils/expansion.hpp>
#include <libcore/skeleton_utils/sampling.hpp>
#include <libcore/skeleton_utils/facet_utils.hpp>
#include <libcore/skeleton_utils/frontier_utils.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

namespace libcore
{
    void skeletonExpansion(const Config& config, SharedVars& vars) {
        Eigen::Vector3d startPt = vars.startPt;
        std::cout << "Skeleton expansion started at: " 
                  << startPt.transpose() << std::endl;

        std::cout << "Generating samples on unit sphere." << std::endl;
        genSamplesOnShape(config, vars);
        std::cout << "Generated " 
                  << vars.sample_directions.size() << " sample directions." << std::endl;

        std::cout << "Identifying first black and white facets." << std::endl;
        identifyBwFacets(vars.sample_directions, vars.bw_facets_directions);
        std::cout << "Identified " 
                  << vars.bw_facets_directions.size() << " black and white facets." << std::endl;

        setStartPt(startPt, config, vars);

        FrontierPtr cur_frontier;

        while (!vars.pending_frontiers.empty()) {

            cur_frontier = pendingFrontiersPopFront(vars.pending_frontiers);
            if (cur_frontier->deleted) continue;
            if (cur_frontier == NULL ) continue;

            verifyFrontier(cur_frontier, config, vars);

            if (!cur_frontier->valid) {
                Eigen::Vector3d prev_proj_center = cur_frontier->proj_center;
                Eigen::Vector3d prev_normal      = cur_frontier->outwards_unit_normal;

                cur_frontier->proj_center = cur_frontier->proj_facet->center;

                verifyFrontier(cur_frontier, config, vars);

                if (!cur_frontier->valid) {
                    cur_frontier->proj_center          = prev_proj_center;
                    cur_frontier->outwards_unit_normal = cur_frontier->proj_facet->outwards_unit_normal;
                    verifyFrontier(cur_frontier, config, vars);

                    if (!cur_frontier->valid) {
                        cur_frontier->proj_center = cur_frontier->proj_facet->center;
                        verifyFrontier(cur_frontier, config, vars);

                        if (!cur_frontier->valid) {
                            cur_frontier->outwards_unit_normal = prev_normal;
                            for (FacetPtr candidate_facet : cur_frontier->proj_facet->nbhd_facets) {
                                cur_frontier->proj_center = candidate_facet->center;
                                verifyFrontier(cur_frontier, config, vars);
                                if (!cur_frontier->valid) {
                                    cur_frontier->outwards_unit_normal = candidate_facet->outwards_unit_normal;
                                    verifyFrontier(cur_frontier, config, vars);
                                    if (cur_frontier->valid) break;
                                } else break;
                            }
                        }
                    }
                }
            }
            
            if (cur_frontier->valid) {
                if (!processFrontier(cur_frontier, config, vars)) {
                }
            }
        }
    }

    void setStartPt(Eigen::Vector3d& startPt, const Config config, SharedVars& vars) {
        NodePtr start_node = std::make_shared<Node>(startPt, nullptr);
        initNode(start_node, config, vars);
    }

    bool initNode(NodePtr curNodePtr, const Config& config, SharedVars& vars) {
        if (!checkWithinBbx(curNodePtr->coord,vars.bbx_min,vars.bbx_max)) { // check if the point is within the bounding box
            return false;
        }
        
        if (!checkFloor(curNodePtr, config.max_ray_length, config.max_height_diff, config, vars)) { // check if the point is on the floor ToDO: check height
            return false;
        }

        if (!curNodePtr->isGate) {
            genBlackAndWhiteVertices(
                curNodePtr,
                config,
                vars
            ); // Generate black and white vertices for the node

            if (curNodePtr->black_vertices.size() < 4) return false;

            centralizeNodePos(curNodePtr); // Centralize the node position according to the black vertices

            double radius = getNodeRadius(curNodePtr); // avg distance to all vertices
            if (radius < config.min_node_radius && 
                curNodePtr->white_vertices.empty()) return false; // if the radius is too small and there are no white vertices, discard the node

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

            findFlowBack(curNodePtr, config, vars); // ToDo figure out what this does

            identifyFacets(curNodePtr, vars.bw_facets_directions);

            identifyFrontiers(curNodePtr, config, vars);

            addFacetsToPcl(curNodePtr,config.resolution, vars.kdtreesForPolys); // add the facets as points to the kdtrees so collision detection can be done on them
        }

        recordNode(curNodePtr, vars);

        return true;
    }

    void findFlowBack(NodePtr node, const Config& config, SharedVars& vars) {
        if (node->seed_frontier == NULL) return;

        int size = vars.loop_candidate_frontiers.size();
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
                config.search_margin
            )) continue;

            FrontierPtr loop_ftr = findFlowBackFrontier(
                v->coord, 
                v->collision_node_index,
                vars.center_NodeList,
                config.max_ray_length,
                config.search_margin
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
            if (((int)flow_back_frontier_log.at(i).size() < config.min_flowback_creation_threshold) && 
                (getVerticesRadius(flow_back_frontier_log.at(i)) < config.min_flowback_creation_radius_threshold)
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
            if (!checkSegClear(node->coord, end_pt_on_frontier, config, vars) ||
                !checkSegClear(flowback_frontier->master_node->coord, end_pt_on_frontier, config, vars)
            ) continue;

            // Bad loop: loop only contains 4 nodes
            if (config.bad_loop_setting) {
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
                if (initNode(new_gate, config, vars)) {
                    flowback_frontier->gate_node = new_gate;
                    flowback_frontier->master_node->connected_nodes.push_back(new_gate);
                    new_gate->connected_nodes.push_back(flowback_frontier->master_node);
                    node->connected_nodes.push_back(new_gate);
                    new_gate->connected_nodes.push_back(node);
                } else continue;
            } else if (flowback_frontier->gate_node->rollbacked) {
                flowback_frontier->gate_node->rollbacked = false;
                recordNode(flowback_frontier->gate_node, vars);
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

    void recordNode(NodePtr new_node, SharedVars& vars) {
        std::cout << "Recording node at: " 
                  << new_node->coord.transpose() << std::endl;
        new_node->id = vars.NodeList.size();
        vars.NodeList.push_back(new_node);
        vars.nodes_pcl.points.push_back(pcl::PointXYZ(
            new_node->coord(0), new_node->coord(1), new_node->coord(2))
        );

        if (!new_node->isGate) {
            int index = vars.center_NodeList.size();
            new_node->index = index;
            vars.center_NodeList.push_back(new_node);
        }

        // Visualize the node
        if (vars.vis.isInitialized()) {
            vars.vis.EnqueueNode(new_node);
            // wait for input for debugging
            //std::cin.get();
        }
    }

    bool processFrontier(FrontierPtr curFtrPtr, const Config& config, SharedVars& vars) {
        // Gate node: midpoint of frontier
        NodePtr gate;

        if (curFtrPtr->gate_node == NULL) {
            gate = std::make_shared<Node>(curFtrPtr->proj_center, curFtrPtr, true);
            curFtrPtr->gate_node = gate;
        } else {
            gate = curFtrPtr->gate_node;
        }

        bool floor = checkFloor(gate, config.max_ray_length, 
                                config.max_height_diff, config, vars);
        bool bbx = checkWithinBbx(gate->coord, vars.bbx_min, vars.bbx_max);
        if (!floor || !bbx) {
            gate->rollbacked = true;
            return false;
        }

        // Center node
        NodePtr new_node = std::make_shared<Node>(curFtrPtr->next_node_pos, curFtrPtr);
        bool init_success = initNode(new_node, config, vars);

        if (init_success) {
            initNode(gate, config, vars);
            // double-sided pointers
            curFtrPtr->master_node->connected_nodes.push_back(gate);
            gate->connected_nodes.push_back(curFtrPtr->master_node);
            gate->connected_nodes.push_back(new_node);
            new_node->connected_nodes.push_back(gate);
        } else {
            new_node->rollbacked = true;
            if (gate->connected_nodes.empty()) {
            gate->rollbacked = true;
            curFtrPtr->valid = false;
            for (auto f : curFtrPtr->facets) {
                f->valid = false;
            }
            }
        }

        return init_success;
    }

} // namespace libcore
