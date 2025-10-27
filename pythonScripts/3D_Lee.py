import numpy as np
import open3d as o3d
from pathlib import Path
from skimage.morphology import skeletonize
from scipy.ndimage import binary_dilation


pcds_path = Path("modular_polygon_generation/libcore/data/maps/")
map_name = "area_1.pcd"
map_vis_name = "area_1_without_ceiling.pcd"

def load_point_cloud(pcd_path: Path) -> o3d.geometry.PointCloud:
    if not pcd_path.exists():
        raise ValueError(f"Point cloud file {pcd_path} does not exist.")
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise ValueError(f"Failed to load point cloud from {pcd_path}")
    print(f"Loaded point cloud from {pcd_path} with {len(pcd.points)} points.")
    return pcd

def rasterize_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float = 0.1, top_cut_off: float = 2.0) -> tuple[np.ndarray, float, np.ndarray]:
    # rasterize the point cloud into a voxel grid
    pts = np.asarray(pcd.points)

    min_bound = pts.min(axis=0) - voxel_size / 2
    max_bound = pts.max(axis=0) + voxel_size / 2
    dims = np.ceil((max_bound - min_bound) / voxel_size).astype(int)

    voxel_grid = np.zeros(dims, dtype=int)
    indices = np.floor((pts - min_bound) / voxel_size).astype(int)
    voxel_grid[indices[:, 0], indices[:, 1], indices[:, 2]] = 1

    print(f"Voxel grid shape: {voxel_grid.shape}")

    # fill all holes:
    # add another layer of 2s underneath the grid
    voxel_grid_padded = np.pad(voxel_grid, pad_width=1, mode='constant', constant_values=2)

    # if a voxel is 0 and the point below is 2, make it 2
    for z in range(1, voxel_grid_padded.shape[2]):
        for y in range(voxel_grid_padded.shape[1]):
            for x in range(voxel_grid_padded.shape[0]):
                if voxel_grid_padded[x, y, z] == 0 and voxel_grid_padded[x, y, z - 1] == 2:
                    voxel_grid_padded[x, y, z] = 2

    # remove the padding
    voxel_grid_filled = voxel_grid_padded[1:-1, 1:-1, 1:-1]

    # convert all 2s to 1s
    voxel_grid_filled[voxel_grid_filled == 2] = 1

    # remove the upper meter
    voxel_grid_result = voxel_grid_filled[:, :, :-int(top_cut_off/voxel_size)]
    return voxel_grid_result, voxel_size, min_bound

def visualize_voxel_grid(voxel_grid: np.ndarray, voxel_size: float, min_bound: np.ndarray):
    # visualize the voxel grid
    occupied_indices = np.argwhere(voxel_grid == 0)
    voxel_centers = occupied_indices * voxel_size + min_bound + voxel_size / 2
    pcd_voxels = o3d.geometry.PointCloud()
    pcd_voxels.points = o3d.utility.Vector3dVector(voxel_centers)

    o3d.visualization.draw_geometries([pcd_voxels])

def skeletonize_voxel_grid(voxel_grid_result: np.ndarray) -> np.ndarray:
    # skeletonize the voxel grid using lee method

    # add a padding of 1s around the grid
    voxel_grid_padded = np.pad(voxel_grid_result, pad_width=1, mode='constant', constant_values=1)
    # smooth the grid by dilating the 0s
    voxel_grid_padded = binary_dilation(voxel_grid_padded == 1, iterations=2, structure=np.ones((3, 3, 3))).astype(int)

    skeleton = skeletonize(voxel_grid_padded == 0, method='lee')
    print(f"Skeleton shape: {skeleton.shape}")

    # skeletonize a second time to thin it out more
    skeleton = skeletonize(skeleton, method='lee')
    return skeleton

def visualize_skeleton(skeleton: np.ndarray, voxel_size: float, min_bound: np.ndarray, pcd: o3d.geometry.PointCloud = None):
    # visualize the skeleton
    skeleton_indices = np.argwhere(skeleton == 1)
    skeleton_centers = skeleton_indices * voxel_size + min_bound + voxel_size / 2
    pcd_skeleton = o3d.geometry.PointCloud()
    pcd_skeleton.points = o3d.utility.Vector3dVector(skeleton_centers)

    if pcd is not None:
        o3d.visualization.draw_geometries([pcd_skeleton, pcd])
    else:
        o3d.visualization.draw_geometries([pcd_skeleton])


def main():
    pcd = load_point_cloud(pcds_path / map_name)
    if map_vis_name != map_name:
        pcd_vis = load_point_cloud(pcds_path / map_vis_name)
    else:
        pcd_vis = pcd

    voxel_grid_result, voxel_size, min_bound = rasterize_point_cloud(pcd, voxel_size=0.1, top_cut_off=2.0)
    print(f"Rasterized voxel grid shape: {voxel_grid_result.shape}")

    visualize_voxel_grid(voxel_grid_result, voxel_size, min_bound)

    skeleton = skeletonize_voxel_grid(voxel_grid_result)
    print(f"Skeleton shape: {skeleton.shape}")

    visualize_skeleton(skeleton, voxel_size, min_bound, pcd_vis)

if __name__ == "__main__":
    main()