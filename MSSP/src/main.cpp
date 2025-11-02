#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <iostream>
#include <limits>


int main() {
    auto vox = xt::load_npy<uint8_t>("../input/voxel_grid.npy");
    auto ske = xt::load_npy<uint8_t>("../input/chen_skeleton.npy");
    auto ske_ids = xt::load_npy<int>("../input/chen_skeleton_id_map.npy");
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

    std::cout << "Sum Voxel: " << xt::sum(vox)() << std::endl;
    std::cout << "Mean Voxel: " << xt::mean(vox)() << std::endl;

    std::cout << "Sum Skeleton: " << xt::sum(ske)() << std::endl;
    std::cout << "Mean Skeleton: " << xt::mean(ske)() << std::endl;

    // check if shape voxel and skeleton is equal
    if (vox.shape() != ske.shape() || ske.shape() != ske_ids.shape()) {
        throw std::runtime_error("Voxel grid, skeleton, skeleton_ids have different shapes!");
    } else {
        std::cout << "Voxel grid, skeleton, and skeleton_ids have the same shape." << std::endl;
    }

    // print M
    std::cout << "M: " << M << std::endl;

    // print M[1,1]
    std::cout << "Voxel size: " << M(1, 1) << std::endl;

    // check if all skeleton voxels are also in voxel grid
    xt::xarray<uint8_t> ske_out_of_vox = ske - (ske & vox);
    xt::xarray<float> ske_dist = xt::ones<float>(ske.shape()) * std::numeric_limits<float>::max();
    // replace all negative values with 0
    int count_missing = xt::sum(ske_out_of_vox)();

    if (count_missing > 0) {
        std::cout << "Not all skeleton voxels are present in the voxel grid! Exactly " << count_missing << " skeleton voxels are missing in the voxel grid. Finding nearest voxel..." << std::endl;
        // Find indices of missing skeleton voxels
        auto out_of_vox_ids = xt::where(ske_out_of_vox);
        auto vox_ids = xt::where(vox);
        for (size_t idx = 0; idx < out_of_vox_ids[0].size(); idx++) {
            int x_out = out_of_vox_ids[0][idx];
            int y_out = out_of_vox_ids[1][idx];
            int z_out = out_of_vox_ids[2][idx];

            float min_dist = std::numeric_limits<float>::max();
            size_t nearest_vox_index = 0;
            bool found = false;

            for (size_t v_idx = 0; v_idx < vox_ids[0].size(); v_idx++) {
                int x_vox = vox_ids[0][v_idx];
                int y_vox = vox_ids[1][v_idx];
                int z_vox = vox_ids[2][v_idx];

                if (ske_ids(x_vox, y_vox, z_vox) != 0) {
                    continue; // skip voxels that already have a ske_id assigned
                }

                float dist = std::sqrt(std::pow(x_out - x_vox, 2) + std::pow(y_out - y_vox, 2) + std::pow(z_out - z_vox, 2));
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_vox_index = v_idx;
                    found = true;
                }
            }

            if (!found) {
                throw std::runtime_error("Could not find nearest voxel for missing skeleton voxel at (" + std::to_string(x_out) + ", " + std::to_string(y_out) + ", " + std::to_string(z_out) + ")!");
            }

            // set the distance and ske_id for the nearest voxel
            int x_nearest = vox_ids[0][nearest_vox_index];;
            int y_nearest = vox_ids[1][nearest_vox_index];;
            int z_nearest = vox_ids[2][nearest_vox_index];

            ske_ids(x_nearest, y_nearest, z_nearest) = ske_ids(x_out, y_out, z_out);
            ske_dist(x_nearest, y_nearest, z_nearest) = min_dist;
        }
    } else {
        std::cout << "All skeleton voxels are present in the voxel grid." << std::endl;
    }

    // recheck missing
    if (xt::count_nonzero(ske_ids)() - count_missing != xt::sum(ske)()) {
        throw std::runtime_error("Some skeleton voxels are still missing in the voxel grid after correction! Exactly " + std::to_string(xt::sum(ske)() - xt::count_nonzero(ske_ids)() + count_missing) + " skeleton voxels are missing.");
    } else {
        std::cout << "All skeleton voxels are now present in the voxel grid." << std::endl;
    }


    std::cout << "Starting distance transform to skeleton..." << std::endl;
    // initialize distance array for skeleton voxels with max float value
    bool changed = true;
    int iteration = 0;

    std::vector<float> n26_dist;

    std::vector<std::vector<int>> n26;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                if (i == 0 && j == 0 && k == 0) continue;
                n26.push_back(std::vector<int>{i, j, k});
                n26_dist.push_back(std::sqrt(i*i + j*j + k*k));
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

    int iteration = 0;
    while (changed) {
        std::cout << "Iteration " << iteration++ << std::endl;
        changed = false;
        for (int i = 1; i < ske.shape()[0] - 1; i++) {
            for (int j = 1; j < ske.shape()[1] - 1; j++) {
                for (int k = 1; k < ske.shape()[2] - 1; k++) {
                    if (vox(i,j,k) == 1) {
                        float min_dist = ske_dist(i, j, k);

                        // iterate over N26 neighbors
                        for (int n = 0; n < n26.size(); n++) {
                            auto offset = n26[n];
                            float dist = n26_dist[n];
                            int ni = i + offset[0];
                            int nj = j + offset[1];
                            int nk = k + offset[2];
                            if (ske_dist(ni, nj, nk) + dist < min_dist) {
                                min_dist = ske_dist(ni, nj, nk) + dist;
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

    std::cout << "Distance transform completed." << std::endl;
    // save the ske_ids array and ske_dist array
    xt::dump_npy("../output/chen_ske_ids.npy", ske_ids);
    xt::dump_npy("../output/chen_ske_dist.npy", ske_dist);
    std::cout << "Saved ske_ids and ske_dist arrays." << std::endl;
}
