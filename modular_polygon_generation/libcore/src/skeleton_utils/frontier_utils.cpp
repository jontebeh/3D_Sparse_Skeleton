#include <libcore/skeleton_utils/frontier_utils.hpp>
#include <libcore/skeleton_utils/facet_utils.hpp>
#include <libcore/skeleton_utils/geometry_utils.hpp>

namespace libcore
{
    bool checkPtOnFrontier(
        FrontierPtr ftr_ptr, 
        Eigen::Vector3d pt, 
        double search_margin
    ) {
        int num_facet = ftr_ptr->facets.size();
        for (int i = 0; i < num_facet; i++) {
            FacetPtr facet = ftr_ptr->facets.at(i);
            if (checkPtOnFacet(facet, pt, search_margin))
            return true;
            else
            continue;
        }
        return false;
    }

    FrontierPtr findFlowBackFrontier(
        Eigen::Vector3d pos, 
        int index,
        const std::vector<NodePtr>& center_NodeList,
        double max_ray_length,
        double search_margin
    ) {
        for (FrontierPtr f : center_NodeList.at(index)->frontiers) {
            if (getDis(pos, f->proj_center) > 2 * max_ray_length)
                continue;
            if (checkPtOnFrontier(f, pos, search_margin))
                return f;
        }

        return NULL;
    }
} // namespace libcore