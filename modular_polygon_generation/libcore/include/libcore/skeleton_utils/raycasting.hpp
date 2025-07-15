#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    std::pair<Eigen::Vector3d, int> raycastOnRawMap(
        Eigen::Vector3d& ray_source,
        Eigen::Vector3d& direction,
        double cut_off_length,
        const Config& config,
        SharedVars& vars);
    
    std::pair<Eigen::Vector3d, int> raycast(
        Eigen::Vector3d ray_source,
        Eigen::Vector3d direction,
        double cut_off_length,
        const Config& config,
        SharedVars& vars);
} // namespace libcore