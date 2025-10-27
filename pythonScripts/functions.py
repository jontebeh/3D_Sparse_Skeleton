import numpy as np
import open3d as o3d
from pathlib import Path
from skimage.morphology import skeletonize
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import networkx as nx

class VisObjVoxel:
    def __init__(self, voxel_grid: np.ndarray, transformation_matrix: np.ndarray, noise: bool = False, invert: bool = False):
        self.voxel_grid = voxel_grid
        self.transformation_matrix = transformation_matrix
        self.noise = noise
        self.invert = invert

    def get_pcd_points(self) -> np.ndarray:
        occupied_indices = np.argwhere(self.voxel_grid == (0 if self.invert else 1))
        occupied_indices_h = np.hstack((occupied_indices, np.ones((occupied_indices.shape[0], 1))))
        occupied_indices = (occupied_indices_h @ self.transformation_matrix.T)[:, :3]
        voxel_size_vec = self.transformation_matrix[:3, :3].diagonal()
        voxel_centers = occupied_indices + voxel_size_vec / 2

        if self.noise:
            voxel_centers += np.random.uniform(-0.05, 0.05, size=voxel_centers.shape)
        
        pcd_voxels = o3d.geometry.PointCloud()
        pcd_voxels.points = o3d.utility.Vector3dVector(voxel_centers)

        return pcd_voxels

class VisObjPCD:
    def __init__(self, pcd: o3d.geometry.PointCloud):
        self.pcd = pcd

class VisObjGraph:
    def __init__(self, graph: nx.Graph):
        self.graph = graph
    
    def get_speheres_and_lines(self, sphere_radius: float = 0.05):
        spheres = []
        node_positions = [data['coords'] for _, data in self.graph.nodes(data=True)]

        for pos in node_positions:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
            sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red
            sphere.translate(pos)
            spheres.append(sphere)

        edges = list(self.graph.edges())
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(node_positions)  # Points for indexing
        line_set.lines = o3d.utility.Vector2iVector(edges)  # Edges as index pairs
        line_set.paint_uniform_color([0.0, 1.0, 0.0])  # Green lines

        return spheres, line_set


def load_point_cloud(pcd_path: Path) -> o3d.geometry.PointCloud:
    """Load a point cloud from a given path.

    Args:
        pcd_path (Path): The path to the point cloud file.

    Raises:
        ValueError: If the point cloud file does not exist or is empty.

    Returns:
        o3d.geometry.PointCloud: The loaded point cloud.
    """
    print(f"Loading point cloud from {pcd_path}...")
    if not pcd_path.exists():
        raise ValueError(f"Point cloud file {pcd_path} does not exist.")
    
    pcd = o3d.io.read_point_cloud(pcd_path)

    if pcd.is_empty():
        raise ValueError(f"Failed to load point cloud from {pcd_path}")
    
    print(f"Loaded point cloud from {pcd_path} with {len(pcd.points)} points.")
    return pcd

def rasterize_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """ Rasterize the point cloud into a voxel grid.

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.
        voxel_size (float, optional): The size of the voxels. Defaults to 0.1.

    Returns:
        tuple[np.ndarray, np.ndarray]: The voxel grid, transformation matrix
    """
    print(f"Rasterizing point cloud with voxel size {voxel_size}...")
    pts = np.asarray(pcd.points) # convert to numpy array

    min_bound = pts.min(axis=0) - voxel_size / 2 # the min coordinates
    max_bound = pts.max(axis=0) + voxel_size / 2 # the max coordinates
    dims = np.ceil((max_bound - min_bound) / voxel_size).astype(int) # the dimensions of the grid

    voxel_grid = np.zeros(dims, dtype=int)
    indices = np.floor((pts - min_bound) / voxel_size).astype(int)
    voxel_grid[indices[:, 0], indices[:, 1], indices[:, 2]] = 1

    transformation_matrix = np.eye(4) # identity matrix (no rotation or scaling)
    transformation_matrix[:3, 3] = min_bound # translate the grid to the origin
    transformation_matrix[:3, :3] *= voxel_size # set the scale according to voxel size

    print(f"Voxel grid created with shape {voxel_grid.shape}")

    return voxel_grid, transformation_matrix

def flood_voxel_grid(voxel_grid: np.ndarray, direction: str = "up") -> np.ndarray:
    """Fill the free voxel cells in the voxel grid from a given direction.

    Args:
        voxel_grid (np.ndarray): The input voxel grid.
        direction (str, optional): The direction to fill the voxels from,
                                    either "up", "down", "left", "right", "front" or "back". Defaults to "up".

    Returns:
        np.ndarray: The filled voxel grid.
    """
    print(f"Filling voxel grid from direction {direction}...")

    if direction not in ["up", "down", "left", "right", "front", "back"]:
        raise ValueError("Direction must be either 'up', 'down', 'left', 'right', 'front' or 'back'.")
    
    direction_map = {
        "up": (0, 0, 1),
        "down": (0, 0, -1),
        "left": (-1, 0, 0),
        "right": (1, 0, 0),
        "front": (0, 1, 0),
        "back": (0, -1, 0)
    }
    dx, dy, dz = direction_map[direction]

    # fill all holes:
    # add a padding of 2s around the grid
    voxel_grid_padded = np.pad(voxel_grid, pad_width=1, mode='constant', constant_values=2)

    # if a voxel is 0 and the direction point is 2, make it 2
    for z in range(1, voxel_grid_padded.shape[2]):
        for y in range(voxel_grid_padded.shape[1]):
            for x in range(voxel_grid_padded.shape[0]):
                z_ = z if dz >= 0 else voxel_grid_padded.shape[2] - z - 1
                y_ = y if dy >= 0 else voxel_grid_padded.shape[1] - y - 1
                x_ = x if dx >= 0 else voxel_grid_padded.shape[0] - x - 1
                if voxel_grid_padded[x_, y_, z_] == 0 and voxel_grid_padded[x_ - dx, y_ - dy, z_ - dz] == 2:
                    voxel_grid_padded[x_, y_, z_] = 2

    # remove the padding
    voxel_grid_filled = voxel_grid_padded[1:-1, 1:-1, 1:-1]

    # convert all 2s to 1s
    voxel_grid_filled[voxel_grid_filled == 2] = 1

    print(f"Voxel grid filled from direction {direction}.")

    return voxel_grid_filled

def invert_voxel_grid(voxel_grid: np.ndarray) -> np.ndarray:
    """Invert the voxel grid, swapping occupied and free spaces.

    Args:
        voxel_grid (np.ndarray): The input voxel grid.

    Returns:
        np.ndarray: The inverted voxel grid.
    """
    print("Inverting voxel grid...")
    inverted_grid = 1 - voxel_grid
    print("Voxel grid inverted.")
    return inverted_grid

def cut_top_of_voxel_grid(voxel_grid_filled: np.ndarray, voxel_size: float, top_cut_off: float) -> np.ndarray:
    """Cut off the top part of the voxel grid.

    Args:
        voxel_grid_filled (np.ndarray): The filled voxel grid.
        voxel_size (float): The size of the voxels.
        top_cut_off (float): The height to cut off.

    Returns:
        np.ndarray: The cut voxel grid.
    """
    print(f"Cutting off the top {top_cut_off} meters of the voxel grid...")
    # remove the upper meter
    voxel_grid_result = voxel_grid_filled[:, :, :-int(top_cut_off/voxel_size)]
    print(f"Voxel grid shape after cutting: {voxel_grid_result.shape}")
    return voxel_grid_result

def visualize(vis_objs: list[VisObjPCD | VisObjVoxel | VisObjGraph] | VisObjPCD | VisObjVoxel | VisObjGraph):
    vis_list = []
    if not isinstance(vis_objs, list):
        vis_objs = [vis_objs]
    for vis_obj in vis_objs:
        if isinstance(vis_obj, VisObjPCD):
            vis_list.append(vis_obj.pcd)
        elif isinstance(vis_obj, VisObjVoxel):
            pcd_voxels = vis_obj.get_pcd_points()
            vis_list.append(pcd_voxels)
        elif isinstance(vis_obj, VisObjGraph):
            spheres, line_set = vis_obj.get_speheres_and_lines()
            vis_list.extend(spheres)
            vis_list.append(line_set)

    o3d.visualization.draw_geometries(vis_list)

def get_sphere(radius: int) -> np.ndarray:
    """Generate a 3D spherical structuring element. (not hollow)

    Args:
        radius (int): The radius of the sphere.

    Returns:
        np.ndarray: A 3D numpy array representing the spherical structuring element.
    """
    L = np.arange(-radius, radius + 1)
    X, Y, Z = np.meshgrid(L, L, L)
    sphere = (X**2 + Y**2 + Z**2) <= radius**2

    return sphere.astype(int)

def skeletonize_voxel_grid(voxel_grid_result: np.ndarray, dilation_size: int = 0) -> np.ndarray:
    """Skeletonize the voxel grid using the Lee method.

    Args:
        voxel_grid_result (np.ndarray): The voxel grid to skeletonize.
        dilation_size (int, optional): The size of the dilation to apply before skeletonization. Defaults to 0.

    Returns:
        np.ndarray: The skeletonized voxel grid.
    """
    # skeletonize the voxel grid using lee method

    # add a padding of 1s around the grid for better skeletonization
    voxel_grid_padded = np.pad(voxel_grid_result, pad_width=1, mode='constant', constant_values=1)
    # smooth the grid by dilating the 0s
    if dilation_size > 0:
        # create a cirlcular dilation structure
        dilation  = get_sphere(dilation_size)
        voxel_grid_padded = binary_dilation(voxel_grid_padded == 1, iterations=1, structure=dilation).astype(int)

    skeleton = skeletonize(voxel_grid_padded == 0, method='lee')
    print(f"Skeleton shape: {skeleton.shape}")

    # skeletonize a second time to thin it out more
    skeleton = skeletonize(skeleton, method='lee')
    print(f"Created skeleton with {np.sum(skeleton)} voxels.")
    return skeleton


def get_graph_from_voxel(voxel_grid: np.ndarray, transformation_matrix: np.ndarray, neighborhood: str = "N6") -> nx.Graph:
    """Convert a voxel grid to a graph representation.

    Args:
        voxel_grid (np.ndarray): The input voxel grid.
        transformation_matrix (np.ndarray): The transformation matrix to apply to the voxel grid.
        neighborhood (str, optional): The type of neighborhood to consider for connectivity.
                                      Options are "N6" (6-connectivity) or "N26" (26-connectivity). Defaults to "N6".

    Returns:
        nx.Graph: The graph representation of the voxel grid.
    """
    print("Converting voxel grid to graph...")
    if neighborhood not in ["N6", "N26"]:
        raise ValueError("Neighborhood must be either 'N6' or 'N26'.")
    
    G = nx.Graph()
    occupied_indices = np.argwhere(voxel_grid == 1)
    M = transformation_matrix
    # transform the indices to the original point cloud coordinates using the transformation matrix
    # add a column of ones to the indices for homogeneous coordinates
    occupied_indices_h = np.hstack((occupied_indices, np.ones((occupied_indices.shape[0], 1))))
    occupied_indices = (occupied_indices_h @ M.T).T[:, :3] # apply the transformation matrix
    voxel_offset = M[:3, :3].diagonal() / 2 # voxel size / 2
    voxel_centers = occupied_indices + voxel_offset # compute voxel centers

    # generate node ids map for fast lookup of neighbors and prevention of rounding errors
    voxel_index_map = {}
    voxel_id_map = {}
    indices_id_map = {}
    for i, coords in enumerate(voxel_centers):
        voxel_index_map[tuple(occupied_indices[i])] = coords
        indices_id_map[tuple(occupied_indices[i])] = i

        voxel_id_map[i] = coords
        G.add_node(i, coords=coords)

    n6_offsets = np.array([
        [-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]
    ])
    n26_offsets = np.array([[i, j, k] for i in [-1, 0, 1]
                             for j in [-1, 0, 1]
                             for k in [-1, 0, 1]
                             if not (i == 0 and j == 0 and k == 0)])

    offsets = n6_offsets if neighborhood == "N6" else n26_offsets

    def distance(a, b):
        return np.linalg.norm(a - b) # euclidean distance

    for np_index, coords in voxel_index_map.items():
        for d in offsets:
            neighbor = tuple(np_index + d)
            if neighbor in voxel_id_map:
                weight = distance(coords, voxel_id_map[neighbor])
                G.add_edge(np_index, indices_id_map[neighbor], weight=weight)


    print(f"Created graph with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges.")
    return G


if __name__ == "__main__":
    print("This is a module with functions for voxel grid processing and visualization. Please import and use the functions as needed.")
    #test_run = input("Do you want to run a test? (y/n): ").lower() == 'y'
    test_run = True
    if test_run:
        pcd_path = Path("./modular_polygon_generation/libcore/data/maps/area_1.pcd")

        pcd = load_point_cloud(pcd_path)

        voxel_grid, transformation_matrix = rasterize_point_cloud(pcd, voxel_size=0.1)
        visualize(VisObjVoxel(voxel_grid, transformation_matrix))
        # fill holes
        voxel_grid_flooded = flood_voxel_grid(voxel_grid, direction='up')
        #visualize_voxel_grid(voxel_grid_flooded, transformation_matrix, invert=False)

        # fill holes from top
        voxel_grid_flooded = flood_voxel_grid(voxel_grid_flooded, direction='down')
        #visualize_voxel_grid(voxel_grid_flooded, transformation_matrix, invert=False)

        voxel_grid_inverted = invert_voxel_grid(voxel_grid_flooded)
        #visualize_voxel_grid(voxel_grid_inverted, transformation_matrix, invert=False)

        # skeletonize
        skeleton = skeletonize_voxel_grid(voxel_grid_flooded, dilation_size=2)
        visualize([VisObjVoxel(skeleton, transformation_matrix), VisObjVoxel(voxel_grid_flooded, transformation_matrix), VisObjPCD(pcd)])

        skeleton_graph = get_graph_from_voxel(skeleton, transformation_matrix, neighborhood="N6")
        visualize(skeleton_graph)