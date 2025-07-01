#pragma once

#include <Eigen/Dense>
#include <libcore/data_types/frontier.hpp>

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
}