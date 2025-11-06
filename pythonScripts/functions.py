import numpy as np
import open3d as o3d
from pathlib import Path
from skimage.morphology import skeletonize
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
import networkx as nx

class VisObjVoxel:
    def __init__(self, voxel_grid: np.ndarray, transformation_matrix: np.ndarray, noise: bool = False, invert: bool = False):
        self.voxel_grid = voxel_grid
        self.transformation_matrix = transformation_matrix
        self.noise = noise
        self.invert = invert

        if voxel_grid.dtype != np.uint8 and voxel_grid.dtype != np.int32:
            self.data_type = "distance_map"
        elif np.array_equal(np.unique(voxel_grid), [0, 1]):
            self.data_type = "binary_map"
        else:
            self.data_type = "id_map"
        
        # color map with voxel_grid shape x 3 for rgb colors
        self.color_map = np.zeros((*voxel_grid.shape, 3), dtype=np.float32)
        self.compute_color_map()

    def get_pcd_points(self) -> np.ndarray:
        occupied_indices = np.argwhere(self.voxel_grid != (1 if self.invert else 0))
        occupied_indices_h = np.hstack((occupied_indices, np.ones((occupied_indices.shape[0], 1))))
        occupied_indices_t = (occupied_indices_h @ self.transformation_matrix.T)[:, :3]
        voxel_size_vec = self.transformation_matrix[:3, :3].diagonal()
        voxel_centers = occupied_indices_t + voxel_size_vec / 2

        if self.noise:
            voxel_centers += np.random.uniform(-0.05, 0.05, size=voxel_centers.shape)
        

        pcd_voxels = o3d.geometry.PointCloud()
        pcd_voxels.points = o3d.utility.Vector3dVector(voxel_centers)

        if self.data_type != "binary_map":
            voxel_colors = []
            for idx in occupied_indices:
                color = self.color_map[tuple(idx)]
                voxel_colors.append(color)
            voxel_colors = np.array(voxel_colors)
            pcd_voxels.colors = o3d.utility.Vector3dVector(voxel_colors)

        return pcd_voxels

    def compute_color_map(self) -> np.ndarray:
        if self.data_type == "binary_map":
            return
        elif self.data_type == "distance_map":
            valid = self.voxel_grid > 0
            vals = self.voxel_grid[valid]
            vmin, vmax = np.percentile(vals, [2, 98])  # ignore extremes
            norm = colors.Normalize(vmin=vmin, vmax=vmax, clip=True)
            cmap = cm.get_cmap('Spectral')

            self.color_map = np.zeros((*self.voxel_grid.shape, 3))
            self.color_map[valid] = cmap(norm(self.voxel_grid[valid]))[:, :3] 
        elif self.data_type == "id_map":
            rng = np.random.default_rng(42)
            unique_ids = np.unique(self.voxel_grid)
            unique_ids = unique_ids[unique_ids != 0]
            rand_colors = rng.random((len(unique_ids), 3))
            color_lut = np.zeros((self.voxel_grid.max() + 1, 3))
            color_lut[unique_ids] = rand_colors
            self.color_map = color_lut[self.voxel_grid]

class VisObjPCD:
    def __init__(self, pcd: o3d.geometry.PointCloud, bounding_box: np.ndarray = None, start_pt: np.ndarray = None):
        self.pcd = pcd
        self.bounding_box = bounding_box
        self.start_pt = start_pt
    
    def get_vis(self):
        vis_list = [self.pcd]
        if self.bounding_box is not None:
            print("Adding bounding box to visualization...")
            line_set = o3d.geometry.LineSet()
            coords = np.zeros((8, 3))
            x_min, y_min, z_min = self.bounding_box[0]
            x_max, y_max, z_max = self.bounding_box[1]
            print(f"Bounding box min: {self.bounding_box[0]}, max: {self.bounding_box[1]}")
            coords[0] = [x_min, y_min, z_min]
            coords[1] = [x_max, y_min, z_min]
            coords[2] = [x_min, y_max, z_min]
            coords[3] = [x_max, y_max, z_min]
            coords[4] = [x_min, y_min, z_max]
            coords[5] = [x_max, y_min, z_max]
            coords[6] = [x_min, y_max, z_max]
            coords[7] = [x_max, y_max, z_max]
            line_set.points = o3d.utility.Vector3dVector(coords)
            lines = []
            for i, coord_1 in enumerate(coords):
                for j, coord_2 in enumerate(coords):
                    # check if any coord_1 is in coord_2
                    if i < j and np.sum(np.abs(coord_1 - coord_2) == 0) != 0:
                        lines.append([i, j])
            line_set.lines = o3d.utility.Vector2iVector(lines)
            vis_list.append(line_set)
        speheres = []
        if self.start_pt is not None:
            print("Adding start point to visualization...")
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            sphere.paint_uniform_color([0.0, 1.0, 0.0])  # Green
            sphere.translate(self.start_pt)
            speheres.append(sphere)
            vis_list.extend(speheres)
        return vis_list

class VisObjGraph:
    def __init__(self, graph: nx.Graph, outlaiers: set = set(), collisions: set = set()):
        self.graph = graph
        self.outlaiers = outlaiers
        self.collisions = collisions

    def get_speheres_and_lines(self, sphere_radius: float = 0.1):
        nodes = []
        for node, data in self.graph.nodes.data():
            if 'coords' not in data:
                raise ValueError(f"Node {node} does not have 'coords' attribute. Other attributes: {data}")
            nodes.append((node, data['coords']))

        # sort nodes by node id and check if they are continuous
        remapped_nodes = {}
        nodes.sort(key=lambda x: x[0])

        for i, (node_id, coords) in enumerate(nodes, start=1):
            if node_id != i:
                print(f"Warning: Node IDs are not continuous. Remapping node {node_id} to {i}.")
                remapped_nodes[node_id] = i

        # get edges
        edges = list(self.graph.edges())
        # remap edges if necessary
        for i, (u, v) in enumerate(edges):
            if u in remapped_nodes:
                u = remapped_nodes[u]
            if v in remapped_nodes:
                v = remapped_nodes[v]
            edges[i] = (u-1, v-1)  # -1 for zero-based indexing

        spheres = []
        if len(self.outlaiers) > 0:
            for outl in self.outlaiers:
                node_coords = self.graph.nodes[outl]['coords']
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
                sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red
                sphere.translate(node_coords)
                spheres.append(sphere)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector([node[1] for node in nodes])  # Points for indexing
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

def load_graph(node_path: Path, edge_path: Path) -> tuple[nx.Graph, dict]:
    """Load a graph from given node and edge files.

    Args:
        node_path (Path): The path to the node file.
        edge_path (Path): The path to the edge file.

    Returns:
        nx.Graph: The loaded graph.
        dict: A mapping from old node IDs to new node IDs.
    """
    print(f"Loading graph from {node_path} and {edge_path}...")
    G = nx.Graph()

    def distance(a, b):
        return np.linalg.norm(a - b) # euclidean distance
    
    remap_node_ids = {}
    next_id = 1

    with open(node_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            node_id = int(parts[0])
            if node_id != next_id:
                remap_node_ids[node_id] = next_id
                node_id = next_id
            coords = np.array([float(coord) for coord in parts[1:4]])
            G.add_node(node_id, coords=coords)
            next_id += 1
    
    with open(edge_path, 'r') as f:
        for line in f:
            # use , as separator
            parts = line.strip().split(',')
            u = int(parts[0]) if int(parts[0]) not in remap_node_ids else remap_node_ids[int(parts[0])]
            v = int(parts[1]) if int(parts[1]) not in remap_node_ids else remap_node_ids[int(parts[1])]

            weight = distance(G.nodes[u]['coords'], G.nodes[v]['coords'])
            G.add_edge(u, v, weight=weight)
    
    print(f"Loaded graph with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges.")
    return G, remap_node_ids

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
            vis_pcd_list = vis_obj.get_vis()
            vis_list.extend(vis_pcd_list)
        elif isinstance(vis_obj, VisObjVoxel):
            pcd = vis_obj.get_pcd_points()
            vis_list.append(pcd)
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

    # remove the padding
    skeleton = skeleton[1:-1, 1:-1, 1:-1]
    print(f"Created skeleton with {np.sum(skeleton)} voxels.")
    return skeleton

def get_graph_from_voxel(voxel_grid: np.ndarray, transformation_matrix: np.ndarray, neighborhood: str = "N26") -> nx.Graph:
    """Convert a voxel grid to a graph representation.

    Args:
        voxel_grid (np.ndarray): The input voxel grid.
        transformation_matrix (np.ndarray): The transformation matrix to apply to the voxel grid.
        neighborhood (str, optional): The type of neighborhood to consider for connectivity.
                                      Options are "N6" (6-connectivity) or "N26" (26-connectivity). Defaults to "N26".

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
    occupied_indices_t = (occupied_indices_h @ M.T)[:, :3] # apply the transformation matrix
    voxel_offset = M[:3, :3].diagonal() / 2 # voxel size / 2
    voxel_centers = occupied_indices_t + voxel_offset # compute voxel centers

    # generate node ids map for fast lookup of neighbors and prevention of rounding errors
    map_np_index_to_coords = {}
    map_np_index_to_id = {}
    map_id_to_coords = {}
    for i, coords in enumerate(voxel_centers):
        map_np_index_to_coords[tuple(occupied_indices[i])] = coords
        map_np_index_to_id[tuple(occupied_indices[i])] = i
        map_id_to_coords[i] = coords
        G.add_node(i+1, coords=coords) # +1 to start node ids from 1
    
    print(f"Total nodes added: {G.number_of_nodes()}")

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

    print("Adding edges...")
    print(f"Processing node {0}/{len(map_np_index_to_id)}")
    for i, (np_index, id) in enumerate(map_np_index_to_id.items()):
        # update progress every 1000 nodes
        if i % 1000 == 0:
            # flash progress bar
            print("\033[F", end='')

            #print progress bar
            print(f"Processing node {i + 1}/{len(map_np_index_to_id)}")

        for d in offsets:
            neighbor_np_index = tuple(np_index + d)
            neighbor_id = map_np_index_to_id.get(neighbor_np_index, None)
            if neighbor_id in map_id_to_coords:
                weight = distance(coords, map_id_to_coords[neighbor_id])
                G.add_edge(id + 1, neighbor_id + 1, weight=weight) # +1 to start node ids from 1


    print(f"Created graph with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges.")
    return G

def get_voxel_from_graph(graph: nx.Graph, transformation_matrix: np.ndarray, grid_shape: tuple[int, int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Convert a graph representation back to a voxel grid.

    Args:
        graph (nx.Graph): The input graph.
        transformation_matrix (np.ndarray): The transformation matrix used to create the graph.
        grid_shape (tuple[int, int, int]): The shape of the voxel grid.

    Returns:
        np.ndarray: The voxel grid representation of the graph.
        np.ndarray: The id map of the graph in voxel grid form.
    """
    print("Converting graph to voxel grid...")
    voxel_grid = np.zeros(grid_shape, dtype=np.uint8)
    id_map = np.full(grid_shape, 0, dtype=np.int32)
    M_inv = np.linalg.inv(transformation_matrix)

    for node, data in graph.nodes.data():
        coords = data['coords']
        # convert coords back to voxel indices
        coords_h = np.hstack((coords, 1))
        voxel_index_t = (coords_h @ M_inv.T)[:3]
        voxel_index = np.floor(voxel_index_t).astype(int)
        x, y, z = voxel_index
        if x < 0 or x >= grid_shape[0]:
            print(f"Warning: Node {node} x index {x} out of bounds.")
            x = np.clip(x, 0, grid_shape[0] - 1)
        if y < 0 or y >= grid_shape[1]:
            print(f"Warning: Node {node} y index {y} out of bounds.")
            y = np.clip(y, 0, grid_shape[1] - 1)
        if z < 0 or z >= grid_shape[2]:
            print(f"Warning: Node {node} z index {z} out of bounds.")
            z = np.clip(z, 0, grid_shape[2] - 1)

        voxel_grid[x, y, z] = 1
        id_map[x, y, z] = node

    print("Graph converted to voxel grid.")
    return voxel_grid, id_map


if __name__ == "__main__":
    print("This is a module with functions for voxel grid processing and visualization. Please import and use the functions as needed.")
    #test_run = input("Do you want to run a test? (y/n): ").lower() == 'y'
    test_run = True
    if test_run:
        pcd_path = Path("./modular_polygon_generation/libcore/data/maps/area_1.pcd")
        chen_graph_path_input = Path("./output/run_1756996692")
        output_path = Path("./MSSP/input/")
        mssp_output_path = Path("./MSSP/output/")

        skeleton_ids = np.load(mssp_output_path / "ske_ids.npy")
        skeleton_dist = np.load(mssp_output_path / "ske_dist.npy")

        chen_skeleton_ids = np.load(mssp_output_path / "chen_ske_ids.npy")
        chen_skeleton_dist = np.load(mssp_output_path / "chen_ske_dist.npy")

        pcd = load_point_cloud(pcd_path)

        voxel_grid, transformation_matrix = rasterize_point_cloud(pcd, voxel_size=0.1)
        #visualize(VisObjVoxel(voxel_grid, transformation_matrix))

        np.save(output_path / "M.npy", transformation_matrix)

        # fill holes
        voxel_grid_flooded = flood_voxel_grid(voxel_grid, direction='up')
        #visualize_voxel_grid(voxel_grid_flooded, transformation_matrix, invert=False)

        # fill holes from top
        voxel_grid_flooded = flood_voxel_grid(voxel_grid_flooded, direction='down')
        #visualize_voxel_grid(voxel_grid_flooded, transformation_matrix, invert=False)

        voxel_grid_inverted = invert_voxel_grid(voxel_grid_flooded)
        #visualize_voxel_grid(voxel_grid_inverted, transformation_matrix, invert=False)

        np.save(output_path / "voxel_grid.npy", voxel_grid_inverted.astype(np.uint8))

        # skeletonize
        skeleton = skeletonize_voxel_grid(voxel_grid_flooded, dilation_size=2)
        #visualize([VisObjVoxel(skeleton, transformation_matrix), VisObjVoxel(voxel_grid_flooded, transformation_matrix), VisObjPCD(pcd)])

        chen_skeleton_graph = load_graph(chen_graph_path_input / "node_list.txt", chen_graph_path_input / "edge_list.txt")

        visualize([VisObjGraph(chen_skeleton_graph), VisObjVoxel(voxel_grid_flooded, transformation_matrix), VisObjPCD(pcd)])

        chen_skeleton_voxel, chen_skeleton_id_map = get_voxel_from_graph(chen_skeleton_graph, transformation_matrix, skeleton.shape)
        visualize([VisObjVoxel(chen_skeleton_voxel, transformation_matrix), VisObjPCD(pcd)])

        np.save(output_path / "chen_skeleton.npy", chen_skeleton_voxel.astype(np.uint8))
        np.save(output_path / "chen_skeleton_id_map.npy", chen_skeleton_id_map.astype(np.int32))

        np.save(output_path / "skeleton.npy", skeleton.astype(np.uint8))
        skeleton_graph = get_graph_from_voxel(skeleton, transformation_matrix, neighborhood="N26")
        visualize([VisObjGraph(skeleton_graph),VisObjVoxel(voxel_grid_flooded, transformation_matrix), VisObjPCD(pcd)])

        visSkelDist = VisObjVoxel(skeleton_dist, np.load(output_path / "M.npy"), noise=True)
        visualize(visSkelDist)

        chen_visSkelDist = VisObjVoxel(chen_skeleton_dist, np.load(output_path / "M.npy"), noise=True)
        visualize(chen_visSkelDist)

        visSkelIds = VisObjVoxel(skeleton_ids, np.load(output_path / "M.npy"), noise=True)
        visualize(visSkelIds)

        chen_visSkelIds = VisObjVoxel(chen_skeleton_ids, np.load(output_path / "M.npy"), noise=True)
        visualize(chen_visSkelIds)