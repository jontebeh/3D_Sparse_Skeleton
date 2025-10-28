#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <iostream>
#include <limits>

int main() {
    try {
        auto vox = xt::load_npy<uint8_t>("../input/voxel_grid.npy");
        auto ske = xt::load_npy<uint8_t>("../input/skeleton.npy");
        auto M = xt::load_npy<double>("../input/M.npy");

        std::cout << "Shape Voxel: ";
        for (auto s : vox.shape()) std::cout << s << " ";
        std::cout << "(" << vox.dimension() << "D)" << std::endl;

        std::cout << "Shape Skeleton: ";
        for (auto s : ske.shape()) std::cout << s << " ";
        std::cout << "(" << ske.dimension() << "D)" << std::endl;

        std::cout << "Shape M: ";
        for (auto s : M.shape()) std::cout << s << " ";
        std::cout << "(" << M.dimension() << "D)" << std::endl;

        std::cout << "Sum: " << xt::sum(vox)() << std::endl;
        std::cout << "Mean: " << xt::mean(vox)() << std::endl;

        // check if shape voxel and skeleton is equal
        if (vox.shape() != ske.shape()) {
            throw std::runtime_error("Voxel grid and skeleton have different shapes!");
        } else {
            std::cout << "Voxel grid and skeleton have the same shape." << std::endl;
        }

        // print M
        std::cout << "M: " << M << std::endl;

        // print M[1,1]
        std::cout << "M[1,1]: " << M(1, 1) << std::endl;

        // check if all skeleton voxels are also in voxel grid
        bool all_voxels_present = true;
        for (int i = 0; i < ske.shape()[0]; i++) {
            for (int j = 0; j < ske.shape()[1]; j++) {
                for (int k = 0; k < ske.shape()[2]; k++) {
                    if (ske(i, j, k) == 1 && vox(i, j, k) == 0) {
                        all_voxels_present = false;
                        break;
                    }
                }
            }
        }


        if (!all_voxels_present) {
            throw std::runtime_error("Not all skeleton voxels are present in the voxel grid!");
        } else {
            std::cout << "All skeleton voxels are present in the voxel grid." << std::endl;
        }

        // give each skeleton voxel an unique id
        xt::xarray<int> ske_ids = xt::zeros<int>(ske.shape());
        int current_id = 1;
        for (int i = 0; i < ske.shape()[0]; i++) {
            for (int j = 0; j < ske.shape()[1]; j++) {
                for (int k = 0; k < ske.shape()[2]; k++) {
                    if (ske(i, j, k) == 1) {
                        ske_ids(i, j, k) = current_id++;
                    }
                }
            }
        }
        // initialize distance array for skeleton voxels with max float value
        
        xt::xarray<float> ske_dist = xt::ones<float>(ske.shape()) * std::numeric_limits<float>::max();
        bool changed = true;
        int iteration = 0;

        std::vector<std::vector<int>> n6 = {
            {1, 0, 0}, {-1, 0, 0},
            {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}
        };

        std::vector<std::vector<int>> n26;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if (i == 0 && j == 0 && k == 0) continue;
                    if (std::abs(i) + std::abs(j) + std::abs(k) == 1) continue; // skip N6 neighbors
                    n26.push_back(std::vector<int>{i, j, k});
                }
            }
        }

        // initialize distance array with 0 vales at skeleton voxels
        for (int i = 0; i < ske.shape()[0]; i++) {
            for (int j = 0; j < ske.shape()[1]; j++) {
                for (int k = 0; k < ske.shape()[2]; k++) {
                    if (ske(i, j, k) == 1) {
                        ske_dist(i, j, k) = 0.0f;
                    }
                }
            }
        }

        while (changed) {
            changed = false;
            for (int i = 1; i < ske.shape()[0] - 1; i++) {
                for (int j = 1; j < ske.shape()[1] - 1; j++) {
                    for (int k = 1; k < ske.shape()[2] - 1; k++) {
                        if (vox(i,j,k) == 1) {
                            float min_dist = ske_dist(i, j, k);
                            // itearte over N6 neighbors
                            for (const auto& offset : n6) {
                                int ni = i + offset[0];
                                int nj = j + offset[1];
                                int nk = k + offset[2];
                                if (ske_dist(ni, nj, nk) + 1.0f < min_dist) {
                                    min_dist = ske_dist(ni, nj, nk) + 1.0f;
                                    ske_ids(i, j, k) = ske_ids(ni, nj, nk);
                                    ske_dist(i, j, k) = min_dist;
                                    changed = true;
                                }
                            }

                            // iterate over N26 neighbors
                            for (const auto& offset : n26) {
                                int ni = i + offset[0];
                                int nj = j + offset[1];
                                int nk = k + offset[2];
                                if (ske_dist(ni, nj, nk) + 1.732f < min_dist) {
                                    min_dist = ske_dist(ni, nj, nk) + 1.732f; // TODO update distance
                                    ske_ids(i, j, k) = ske_ids(ni, nj, nk);
                                    ske_dist(i, j, k) = min_dist;
                                    changed = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < ske.shape()[0]; i++) {
            for (int j = 0; j < ske.shape()[1]; j++) {
                for (int k = 0; k < ske.shape()[2]; k++) {
                    if (vox(i,j,k) == 0) {
                        ske_dist(i, j, k) = 0.0f;
                    }
                    if (ske_dist(i, j, k) == std::numeric_limits<float>::max()) {
                        ske_dist(i, j, k) = -1.0f; // mark unreachable voxels with -1
                    }
                }
            }
        }


        // save the ske_ids array and ske_dist array
        xt::dump_npy("../output/ske_ids.npy", ske_ids);
        xt::dump_npy("../output/ske_dist.npy", ske_dist);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
