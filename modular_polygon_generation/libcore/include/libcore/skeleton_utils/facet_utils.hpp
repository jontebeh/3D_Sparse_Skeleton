#pragma once

#include <libcore/quickhull/QuickHull.hpp>

#include <Eigen/Dense>

#include <libcore/data_types/facet.hpp>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace libcore
{
    void identifyBwFacets(
        std::vector<Eigen::Vector3d> &sample_directions,
        std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions);
    
    bool checkPtOnFacet(
        const FacetPtr facet, 
        const Eigen::Vector3d pt, 
        double search_margin);
    
    void identifyFacets(
        NodePtr node,
        const std::vector<std::vector<Eigen::Vector3d>> &bw_facets_directions
    );

    std::vector<FacetPtr> findGroupFacetsFromVertices(
        NodePtr node,
        std::vector<VertexPtr> group_bv,
        const double z_min,
        const double z_max,
        const double search_margin
    );

    void findNbhdFacets(std::vector<FacetPtr>& facets);

    bool facetOnCeilOrFloor(
        const FacetPtr f,
        const double z_min,
        const double z_max,
        const double search_margin
    );

     void addFacetsToPcl(
        NodePtr nodePtr,
        const double resolution,
        std::vector<std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>>& kdtreesForPolys
    );
} // namespace libcore