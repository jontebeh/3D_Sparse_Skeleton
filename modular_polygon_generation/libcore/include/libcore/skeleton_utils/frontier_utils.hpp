#pragma once

#include <Eigen/Dense>
#include <libcore/data_types/frontier.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    bool checkPtOnFrontier(
        FrontierPtr ftr_ptr, 
        Eigen::Vector3d pt, 
        double search_margin);
    
    FrontierPtr findFlowBackFrontier(
        Eigen::Vector3d pos, 
        int index,
        const std::vector<NodePtr>& center_NodeList,
        double max_ray_length,
        double search_margin);
    
    void identifyFrontiers(
        NodePtr node,
        const Config& config,
        SharedVars& vars
    );
    
    FrontierPtr pendingFrontiersPopFront(std::deque<FrontierPtr>& pending_frontiers);

    void verifyFrontier(
        FrontierPtr ftr_ptr,
        const Config& config,
        SharedVars& vars
    );

    std::vector<FrontierPtr> splitFrontier(
        NodePtr node, 
        std::vector<FacetPtr> group_facets,
        int max_facets_grouped,
        double frontier_split_threshold
    );

    bool initFrontier(FrontierPtr frontier);

    bool compareFrontier(const FrontierPtr f1, const FrontierPtr f2);
}