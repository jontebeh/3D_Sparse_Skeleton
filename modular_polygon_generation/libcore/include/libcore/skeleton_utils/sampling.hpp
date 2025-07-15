#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore{
    void genSamplesOnUnitSphere(const Config& config, SharedVars& vars);

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const Config& config,
        SharedVars& vars
    );
} // namespace libcore