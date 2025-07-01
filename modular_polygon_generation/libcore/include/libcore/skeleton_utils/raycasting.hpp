#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Raycasts from a point in a specified direction and returns the intersection point and status.
     * 
     * This function performs a raycast operation from a given point in a specified direction
     * and returns the intersection point with the map and the status of the raycast.
     * 
     * @param ray_source The starting point of the raycast.
     * @param direction The direction of the raycast.
     * @param cut_off_length The maximum length of the raycast.
     * @param c The context containing the map representation, search margin, and kd-tree for raw map.
     * @return A pair containing the intersection point and the status of the raycast.
     */
    std::pair<Eigen::Vector3d, int> raycastOnRawMap(
        const Eigen::Vector3d& ray_source,
        const Eigen::Vector3d& direction,
        double cut_off_length,
        const RaycastingContext& c
        );
    
    std::pair<Eigen::Vector3d, int> raycast(
        Eigen::Vector3d ray_source, 
        Eigen::Vector3d direction,
        double cut_off_length,
        const RaycastingContext& c);
} // namespace libcore