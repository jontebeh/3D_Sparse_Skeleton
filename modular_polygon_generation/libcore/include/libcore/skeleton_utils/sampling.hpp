#pragma once

#include <Eigen/Dense>

#include <libcore/data_types/node.hpp>
#include <libcore/data_types/contexts.hpp>

namespace libcore{
    /**
     * @brief Generates sample directions on the unit sphere using the Fibonacci sphere method.
     * 
     * @param sample_directions A vector to store the generated sample directions.
     * @param sampling_density The number of sample directions to generate.
     */
    void genSamplesOnUnitSphere(
        const SamplingContext& c);

    void genBlackAndWhiteVertices(
        NodePtr nodePtr,
        const SamplingContext& c);
} // namespace libcore