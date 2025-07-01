#pragma once

#include <libcore/quickhull/QuickHull.hpp>

#include <Eigen/Dense>

#include <libcore/data_types/facet.hpp>

namespace libcore
{
    void identifyBwFacets(
        std::vector<Eigen::Vector3d> &sample_directions,
        std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions);
    
    bool checkPtOnFacet(
        FacetPtr facet, 
        Eigen::Vector3d pt, 
        double search_margin);
} // namespace libcore