#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore{
    double superquadricFunction(
        const Eigen::Vector3d& direction,
        double width,
        double height,
        double depth,
        double sharpness
    );

    void genSamplesOnShape(const Config& config, SharedVars& vars);

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const Config& config,
        SharedVars& vars
    );
} // namespace libcore