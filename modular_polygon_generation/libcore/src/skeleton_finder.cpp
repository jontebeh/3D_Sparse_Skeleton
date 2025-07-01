#include <libcore/skeleton_finder.hpp>
#include <libcore/skeleton_utils/map_interface.hpp>
#include <libcore/skeleton_utils/expansion.hpp>
#include <iostream>
#include <chrono>

namespace libcore {
    SkeletonFinder::SkeletonFinder() {}

    SkeletonFinder::~SkeletonFinder() {}

    void SkeletonFinder::init(const SkeletonParameters& params) {
        std::cout << "Initializing SkeletonFinder with parameters." << std::endl;
        setParameters(params);

        std::cout << "Setting up maps for skeleton finding." << std::endl;
        map_pcl = getMap();

        MapContext map_context;//ToDo

        std::cout << "Pre-processing maps for visualization and search." << std::endl;
        preProcessMaps(map_pcl, raw_map_pcl, vis_map_pcl, map_context);

        if (map_context.map_representation == 0) {
            kdtreeForRawMap.setInputCloud(raw_map_pcl.makeShared());
        }

        std::cout << "Generating skeleton..." << std::endl;
        auto begin = std::chrono::high_resolution_clock::now();

        Eigen::Vector3d start;
        start << _params.start_x, _params.start_y, _params.start_z;
        ExpansionContext expansion_context;//ToDo
        skeletonExpansion(expansion_context);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        std::cout << "Skeleton generation completed in " << duration.count() << " ms." << std::endl;

    }

    void SkeletonFinder::setParameters(const SkeletonParameters& params) {
        _params = params;
    }
}  // namespace libcore