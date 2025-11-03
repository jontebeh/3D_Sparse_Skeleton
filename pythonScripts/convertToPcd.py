
import open3d as o3d
import numpy as np
import os
import glob
from tqdm import tqdm
from pathlib import Path

def load_all_area_points(base_folder, area_list, filters):
    """Loads and merges all XYZRGB points from areas specified in the config."""
    all_annotation_files = []

    for area_name in area_list:
        area_path = os.path.join(base_folder, area_name)
        annotation_files = glob.glob(os.path.join(area_path, "*", "Annotations", "*.txt"))
        for annotation_file in annotation_files:
            if any(f in annotation_file for f in filters):
                # skip files that match any filter
                continue
            all_annotation_files.append(annotation_file)

    if not all_annotation_files:
            raise RuntimeError("No annotation files found for the specified areas.")

    all_points = []
    for file_path in tqdm(all_annotation_files, desc="Loading S3DIS Points"):
        try:
            data = np.loadtxt(file_path)  # x y z r g b
        except Exception as e:
            print(f"Error loading file {file_path}: {e}")
            continue
        all_points.append(data[:, :6])  # Only XYZRGB

    print("loaded all points")

    merged_data = np.vstack(all_points)
    print(f"Merged data shape: {merged_data.shape}")
    return merged_data

def create_open3d_pcd(data):
    """Converts numpy array with XYZRGB to Open3D PointCloud."""
    print("Creating Open3D PointCloud from data")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:, :3])
    print("Setting colors for PointCloud")
    pcd.colors = o3d.utility.Vector3dVector(data[:, 3:] / 255.0)
    print("PointCloud created successfully")
    return pcd

if __name__ == "__main__":
    # Replace with your S3DIS dataset root
    dataset_root = Path("../datasets/datasets/S3DIS/")
    outpath = Path("./modular_polygon_generation/libcore/data/maps/")

    area_list = [
        "Area_5",
    ]

    filters = []

    for area in area_list:
        print(f"Processing {area}...")
        data = load_all_area_points(dataset_root, [area], filters)
        pcd = create_open3d_pcd(data)

        o3d.io.write_point_cloud(outpath / f"{area}.pcd", pcd, write_ascii=False)

        #o3d.visualization.draw_geometries([pcd], window_name=f"S3DIS Area: {area}", width=1600, height=900)

