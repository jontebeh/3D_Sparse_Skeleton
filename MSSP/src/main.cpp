#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <iostream>
#include <limits>
#include <filesystem>

void progress_bar(float progress) {
    int barWidth = 70;
    std::cout << "[";
    int pos = static_cast<int>(barWidth * progress);
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

void find_closest(xt::xarray<int32_t>& id_map, xt::xarray<float>& dist_map, const xt::xarray<int64_t>& vox_map) {
    // skeleton = id_map > 0
    xt::xarray<uint8_t> ske = id_map > 0;
    // out_of_vox = ske & vox_map xor ske
    xt::xarray<uint8_t> out_of_vox = (ske & vox_map) ^ ske;

    if (xt::sum(out_of_vox)() == 0) {
        std::cout << "All skeleton voxels are present in the voxel grid." << std::endl;
        return;
    } else {
        std::cout << xt::sum(out_of_vox)() << " skeleton voxels are missing in the voxel grid! Finding nearest voxel..." << std::endl;
    }

    // Find indices of missing skeleton voxels
    auto out_of_vox_ids = xt::where(out_of_vox);
    auto vox_ids = xt::where(vox_map);

    for (size_t idx = 0; idx < out_of_vox_ids[0].size(); idx++) {
        progress_bar(static_cast<float>(idx) / static_cast<float>(out_of_vox_ids[0].size()));
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

            if (id_map(x_vox, y_vox, z_vox) != 0) {
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

        id_map(x_nearest, y_nearest, z_nearest) = id_map(x_out, y_out, z_out);
        dist_map(x_nearest, y_nearest, z_nearest) = min_dist;
    }
    progress_bar(1.0);
    std::cout << std::endl;

    // recheck missing
    if (xt::sum(ske)() + xt::sum(out_of_vox)() != xt::count_nonzero(id_map)()) {
        throw std::runtime_error("Some skeleton voxels are still missing in the voxel grid after correction! Exactly " + std::to_string(xt::sum(ske)() + xt::sum(out_of_vox)() - xt::count_nonzero(id_map)()) + " skeleton voxels are missing.");
    } else {
        std::cout << "All skeleton voxels are now present in the voxel grid." << std::endl;
    }
}

void dist_transform(xt::xarray<int32_t>& id_map, xt::xarray<float>& dist_map, const xt::xarray<int64_t>& vox_map) {
    std::cout << "Starting distance transform to skeleton..." << std::endl;
    // initialize distance array for skeleton voxels with max float value
    bool changed = true;

    auto shape = dist_map.shape();
    std::vector<float> n26_dist;
    std::vector<std::vector<int>> n26;

    // generate N26 neighbor offsets and their distances
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                if (i == 0 && j == 0 && k == 0) continue;
                n26.push_back(std::vector<int>{i, j, k});
                n26_dist.push_back(std::sqrt(i*i + j*j + k*k));
            }
        }
    }

    // initialize distance array with 0 values at skeleton voxels
    for (size_t i = 0; i < shape[0]; i++) {
        for (size_t j = 0; j < shape[1]; j++) {
            for (size_t k = 0; k < shape[2]; k++) {
                if (id_map(i, j, k) > 0 && dist_map(i, j, k) == std::numeric_limits<float>::max()) {
                    dist_map(i, j, k) = 0.0f;
                }
            }
        }
    }

    // directions: abs(value) is axis order (1=x/i, 2=y/j, 3=z/k), sign is iteration direction
    // customize this list as you like:
    const std::vector<std::array<int,3>> sweeps = {
        {+1,+2,+3}, // x,y,z
        {+2,+1,+3}, // y,x,z
        {+3,+2,+1}, // z,y,x
        {-1,+2,-3}, // -x,y,-z
        {-2,+1,-3}, // -y,x,-z
        {+1,-3,+2}, // x,-z,y
    };

    auto sizeX = static_cast<int>(shape[0]);
    auto sizeY = static_cast<int>(shape[1]);
    auto sizeZ = static_cast<int>(shape[2]);

    // helper to get loop bounds from sign
    auto bounds = [](int n, int sign) {
        // skip borders: [1 .. n-2]
        if (sign > 0) return std::tuple<int,int,int>(1, n-1, +1);     // i = 1 .. n-2
        else          return std::tuple<int,int,int>(n-2, 0,  -1);    // i = n-2 .. 1
    };

    // one sweep with a given {±1,±2,±3}
    auto do_sweep = [&](const std::array<int,3>& dir, bool& changed) {
        // decode order & signs
        int ax0 = std::abs(dir[0]) - 1; int s0 = (dir[0] > 0) ? +1 : -1;
        int ax1 = std::abs(dir[1]) - 1; int s1 = (dir[1] > 0) ? +1 : -1;
        int ax2 = std::abs(dir[2]) - 1; int s2 = (dir[2] > 0) ? +1 : -1;

        int sizes[3] = { sizeX, sizeY, sizeZ };

        // outer/inner loop ranges, according to chosen order/sign
        auto [b0, e0, st0] = bounds(sizes[ax0], s0);
        auto [b1, e1, st1] = bounds(sizes[ax1], s1);
        auto [b2, e2, st2] = bounds(sizes[ax2], s2);

        // iterate in the permuted order, but write to (i,j,k) in canonical axis slots
        for (int u = b0; u != e0; u += st0) {
            progress_bar(static_cast<float>(u) / (static_cast<float>(sizes[ax0])));
            for (int v = b1; v != e1; v += st1) {
                for (int w = b2; w != e2; w += st2) {
                    // cur[0]=i, cur[1]=j, cur[2]=k (canonical x,y,z indices)
                    int cur[3];
                    cur[ax0] = u;
                    cur[ax1] = v;
                    cur[ax2] = w;

                    int i = cur[0], j = cur[1], k = cur[2];

                    if (vox_map(i, j, k) == 1) {
                        float min_dist = dist_map(i, j, k);

                        // N26 relaxation
                        for (int n = 0; n < static_cast<int>(n26.size()); ++n) {
                            const auto& offset = n26[n];   // e.g., {dx,dy,dz} in canonical axes
                            float d = n26_dist[n];

                            int ni = i + offset[0];
                            int nj = j + offset[1];
                            int nk = k + offset[2];

                            float cand = dist_map(ni, nj, nk) + d;
                            if (cand < min_dist) {
                                min_dist = cand;
                                id_map(i, j, k) = id_map(ni, nj, nk);
                                dist_map(i, j, k) = min_dist;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
        progress_bar(1.0);
        std::cout << std::endl;
    };

    // run until convergence over a full cycle of sweeps
    int iteration = 0;
    bool any_changed = true;
    while (any_changed) {
        std::cout << "Iteration " << iteration++ << std::endl;
        any_changed = false;
        for (const auto& s : sweeps) {
            bool changed_this_sweep = false;
            do_sweep(s, changed_this_sweep);
            any_changed = any_changed || changed_this_sweep;
        }
    }


    /*
    int iteration = 0;
    while (changed) {
        std::cout << "Iteration " << iteration++ << std::endl;
        changed = false;
        for (size_t i = 1; i < shape[0] - 1; i++) {
            for (size_t j = 1; j < shape[1] - 1; j++) {
                for (size_t k = 1; k < shape[2] - 1; k++) {
                    if (vox_map(i,j,k) == 1) {
                        float min_dist = dist_map(i, j, k);

                        // iterate over N26 neighbors
                        for (int n = 0; n < n26.size(); n++) {
                            auto offset = n26[n];
                            float dist = n26_dist[n];
                            int ni = i + offset[0];
                            int nj = j + offset[1];
                            int nk = k + offset[2];
                            if (dist_map(ni, nj, nk) + dist < min_dist) {
                                min_dist = dist_map(ni, nj, nk) + dist;
                                id_map(i, j, k) = id_map(ni, nj, nk);
                                dist_map(i, j, k) = min_dist;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
    }
    */

    // finalize: set dist to 0 for voxels that are not in vox_map, and -1 for unreachable voxels
    for (size_t i = 0; i < shape[0]; i++) {
        for (size_t j = 0; j < shape[1]; j++) {
            for (size_t k = 0; k < shape[2]; k++) {
                if (vox_map(i,j,k) == 0) {
                    dist_map(i, j, k) = 0.0f;
                } else if (dist_map(i, j, k) == std::numeric_limits<float>::max()) {
                    dist_map(i, j, k) = -1.0f; // mark unreachable voxels with -1
                }
            }
        }
    }

    std::cout << "Distance transform completed." << std::endl;
}

bool process_run(std::filesystem::path run_path, xt::xarray<int64_t>& vox_map) {
    std::cout << "Processing run directory: " << run_path << std::endl;
    std::filesystem::path id_map_path = run_path / "chen_voxel_id_map.npy";

    xt::xarray<int32_t> id_map = xt::load_npy<int32_t>(id_map_path);

    std::cout << "Shape Voxel: ";
    for (auto s : vox_map.shape()) std::cout << s << " ";
    std::cout << "(" << vox_map.dimension() << "D)" << std::endl;

    std::cout << "Shape Skeleton: ";
    for (auto s : id_map.shape()) std::cout << s << " ";
    std::cout << "(" << id_map.dimension() << "D)" << std::endl;

    if (vox_map.shape() != id_map.shape()) {
        std::cerr << "Error: Voxel map and skeleton map have different shapes!" << std::endl;
        return false;
    }

    xt::xarray<float> dist_map = xt::ones<float>(vox_map.shape()) * std::numeric_limits<float>::max();

    std::cout << "Number of Voxels: " << xt::sum(vox_map)() << std::endl;

    std::cout << "Number of Nodes: " << xt::sum(id_map > 0)() << std::endl;

    find_closest(id_map, dist_map, vox_map);
    dist_transform(id_map, dist_map, vox_map);

    // save the ske_ids array and ske_dist array
    // check if the file already exists
    if (!std::filesystem::exists(run_path / "chen_id_map_full.npy")) {
        xt::dump_npy(run_path / "chen_id_map_full.npy", id_map);
        std::cout << "Saved full ske_ids array." << std::endl;
    }

    if (!std::filesystem::exists(run_path / "chen_dist_map.npy")) {
        xt::dump_npy(run_path / "chen_dist_map.npy", dist_map);
        std::cout << "Saved ske_dist array." << std::endl;
    }
    return true;
}

bool recursive_process(std::filesystem::path base_path, xt::xarray<int64_t>& vox_map) {
    // check if base_path is a directory
    if (!std::filesystem::is_directory(base_path)) {
        std::cerr << "Error: " << base_path << " is not a directory!" << std::endl;
        return false;
    }

    // check if base_path contains run_
    if (base_path.filename().string().find("run_") != std::string::npos) {
        try
        {
            process_run(base_path, vox_map);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    // get subdirectories
    for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
        if (entry.is_directory()) {
            recursive_process(entry.path(), vox_map);
        }
    }
    return true;
}

int main() {
    std::filesystem::path test_path = std::filesystem::path("../../output/tests/");
    std::filesystem::path vox_map_path = std::filesystem::path("../../data/voxel_maps/area_1/area_1_size_0_1_voxel_grid_inverted.npy");
    xt::xarray<int64_t> vox_map = xt::load_npy<int64_t>(vox_map_path);

    recursive_process(test_path, vox_map);
}
