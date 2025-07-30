#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Performs raycasting on the raw map to find the first point along a ray.
     * 
     * This function casts a ray from a source point in a specified direction and returns the first point
     * encountered within a specified cut-off length. If no point is found, it returns the original ray source
     * with a flag indicating no point was found.
     * 
     * @param ray_source The starting point of the ray.
     * @param direction The direction of the ray.
     * @param cut_off_length The maximum length of the ray to search for points.
     * @param config The configuration settings for the raycasting operation.
     * @param vars Shared variables containing the k-d tree and other necessary data.   
     * 
     * @return A pair containing the found point (or the original ray source if no point was found) and an integer flag:
     *         -1 if a point was found,
     *         -2 if no point was found within the cut-off length,
     *         0 if the map representation is not a point cloud.
     */
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