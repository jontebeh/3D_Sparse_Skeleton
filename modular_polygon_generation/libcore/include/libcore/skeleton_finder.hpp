#pragma once

#include <libcore/data_types/graph_types.hpp>
#include <libcore/data_types/contexts.hpp>
#include <libcore/skeleton_utils/map_interface.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace libcore {

class SkeletonFinder {
public:
    /**
     * @brief Default constructor for SkeletonFinder.
     */
    SkeletonFinder();

    /**
     * @brief Destructor for SkeletonFinder.
     */
    ~SkeletonFinder();

    void init(std::string config_file);

    /**
     * @brief Initializes the skeleton finder with the given parameters.
     * 
     */
    void init();

private:
    /**
     * @brief Reads the configuration file and initializes the Config object.
     * 
     * @param config_file Path to the configuration file.
     * @return Config The configuration settings read from the file.
     */
    Config readConfigFile(const std::string& config_file);
};

} // namespace libcore
