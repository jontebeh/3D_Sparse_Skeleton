#pragma once

#include <Eigen/Dense>
#include <chrono>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    void skeletonExpansion(const Config& config, SharedVars& vars);

    void setStartPt(Eigen::Vector3d& startPt, const Config config, SharedVars& vars);

    bool initNode(NodePtr curNodePtr, const Config& config, SharedVars& vars);

    bool processFrontier(FrontierPtr curFtrPtr, const Config& config, SharedVars& vars);

    void recordNode(NodePtr new_node, SharedVars& vars);

    void findFlowBack(NodePtr node, const Config& config, SharedVars& vars);

} // namespace libcore
