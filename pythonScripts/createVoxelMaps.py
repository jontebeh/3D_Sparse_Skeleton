from functions import *
from pathlib import Path
import timeit

def main():
    pcds_path = Path("modular_polygon_generation/libcore/data/maps/")
    output_path = Path("./data/voxel_maps/")
    # create output directory if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)
    map_list = [
        #("area_1.pcd", 0.01),
        ("area_1.pcd", 0.05),
        ("area_1.pcd", 0.1),
        ("area_1.pcd", 0.15),
        ("area_1.pcd", 0.2),
        ("Area_2.pcd", 0.1),
        ("Area_3.pcd", 0.1),
        ("Area_4.pcd", 0.1),
        ("Area_5.pcd", 0.1),
        ("Area_6.pcd", 0.1),
    ]

    for map_name, voxel_size in map_list:
        print(f"Processing {map_name} with voxel size {voxel_size}")

        stats = {}
        start_time = timeit.default_timer()
        pcd_path = pcds_path / map_name
        pcd = load_point_cloud(pcd_path)
        stats['load_time'] = timeit.default_timer() - start_time

        start_time = timeit.default_timer()
        voxel_grid, M = rasterize_point_cloud(pcd, voxel_size=voxel_size)
        stats['rasterize_time'] = timeit.default_timer() - start_time

        start_time = timeit.default_timer()
        voxel_grid_flooded = flood_voxel_grid(voxel_grid, direction="up")
        voxel_grid_flooded = flood_voxel_grid(voxel_grid_flooded, direction="down")
        stats['flood_time'] = timeit.default_timer() - start_time

        start_time = timeit.default_timer()
        voxel_grid_inverted = invert_voxel_grid(voxel_grid_flooded)
        stats['invert_time'] = timeit.default_timer() - start_time

        start_time = timeit.default_timer()
        skeleton = skeletonize_voxel_grid(voxel_grid_flooded, int(0.1 * 2.0 / voxel_size))
        stats['skeleton_time'] = timeit.default_timer() - start_time
        stats['skeleton_dilations'] = int(0.1 * 2.0 / voxel_size)

        # save np files
        file_start =  f"{map_name[:-4]}_size_{voxel_size}_"
        np.save(output_path / f"{file_start}voxel_grid.npy", voxel_grid)
        np.save(output_path / f"{file_start}voxel_grid_flooded.npy", voxel_grid_flooded)
        np.save(output_path / f"{file_start}voxel_grid_inverted.npy", voxel_grid_inverted)
        np.save(output_path / f"{file_start}skeleton.npy", skeleton)
        np.save(output_path / f"{file_start}M.npy", M)
        # save stats
        with open(output_path / f"{file_start}stats.txt", 'w') as f:
            for key, value in stats.items():
                f.write(f"{key}: {value}\n")

if __name__ == "__main__":
    main()

