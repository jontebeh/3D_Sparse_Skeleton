#pragma once

#include <Eigen/Dense>
#include <chrono>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore
{
    /**
     * @brief Starts the skeleton expansion algorithm from a given starting point.
     * 
     * @param c The context containing parameters for the expansion.
     */
    void skeletonExpansion(const ExpansionContext& c);

    /**
     * @brief Initializes a node with the given coordinates.
     * 
     * This function initializes a node and checks if it is within the bounding box and on the floor.
     * 
     * @param curNodePtr Pointer to the node to be initialized.
     * @param c The context containing parameters for the expansion.
     */
    void setStartPt(Eigen::Vector3d& startPt, 
                    const ExpansionContext& c);

    /**
     * @brief Initializes a node.
     * 
     * This function checks if the node is within the bounding box and on the floor, and generates black and white vertices if it is not a gate.
     * 
     * @param curNodePtr Pointer to the node to be initialized.
     * @param c The context containing parameters for the expansion.
     * @return true if the node is successfully initialized, false otherwise.
     */
    bool initNode(NodePtr curNodePtr, 
                  const ExpansionContext& c);

} // namespace libcore
