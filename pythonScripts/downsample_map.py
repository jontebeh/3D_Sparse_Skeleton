from functions import *
import open3d as o3d

pcd_path = Path("./modular_polygon_generation/libcore/data/maps/area_1.pcd")
output_path = Path("./modular_polygon_generation/libcore/data/maps/")
pcd = load_point_cloud(pcd_path)
print(f"Number of points in original point cloud: {len(pcd.points)}")

area_1_downsampled_0_5 = pcd.voxel_down_sample(voxel_size=0.5)
print(f"Number of points in downsampled point cloud (0.5): {len(area_1_downsampled_0_5.points)}")
visualize(VisObjPCD(area_1_downsampled_0_5))
o3d.io.write_point_cloud(str(output_path / "area_1_downsampled_0_5.pcd"), area_1_downsampled_0_5, write_ascii=False)

area_1_downsampled_0_2 = pcd.voxel_down_sample(voxel_size=0.2)
print(f"Number of points in downsampled point cloud (0.2): {len(area_1_downsampled_0_2.points)}")
visualize(VisObjPCD(area_1_downsampled_0_2))
o3d.io.write_point_cloud(str(output_path / "area_1_downsampled_0_2.pcd"), area_1_downsampled_0_2, write_ascii=False)

area_1_downsampled_0_1 = pcd.voxel_down_sample(voxel_size=0.1)
print(f"Number of points in downsampled point cloud (0.1): {len(area_1_downsampled_0_1.points)}")
visualize(VisObjPCD(area_1_downsampled_0_1))
o3d.io.write_point_cloud(str(output_path / "area_1_downsampled_0_1.pcd"), area_1_downsampled_0_1, write_ascii=False)

area_1_downsampled_0_05 = pcd.voxel_down_sample(voxel_size=0.05)
print(f"Number of points in downsampled point cloud (0.05): {len(area_1_downsampled_0_05.points)}")
visualize(VisObjPCD(area_1_downsampled_0_05))
o3d.io.write_point_cloud(str(output_path / "area_1_downsampled_0_05.pcd"), area_1_downsampled_0_05, write_ascii=False)

area_1_downsampled_0_01 = pcd.voxel_down_sample(voxel_size=0.01)
print(f"Number of points in downsampled point cloud (0.01): {len(area_1_downsampled_0_01.points)}")
visualize(VisObjPCD(area_1_downsampled_0_01))
o3d.io.write_point_cloud(str(output_path / "area_1_downsampled_0_01.pcd"), area_1_downsampled_0_01, write_ascii=False)