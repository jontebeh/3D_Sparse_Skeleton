from skimage.draw import line_nd
from pathlib import Path
import numpy as np

area_path = Path("./data/voxel_maps/area_1")
run_path = Path("./output/tests/area_1_parameters/min_wall_dist/run_1762273164")


voxel_grid = np.load(area_path / "area_1_size_0_1_voxel_grid.npy")
M = np.load(area_path / "area_1_size_0_1_M.npy")
M_inv = np.linalg.inv(M)


def process_run(run_path: Path):
    print(f"Processing run {run_path.name}")
    nodes = {}
    node_path = run_path / "node_list.txt"
    edge_path = run_path / "edge_list.txt"
    graph_stats_path = run_path / "graph_stats.json"

    if not node_path.exists() or not graph_stats_path.exists():
        return 0
    
    # load nodes
    nodes_coords = []
    with open(node_path, "r") as f:
        c = 0
        for line in f:
            parts = line.strip().split(",")
            node_id = int(parts[0])
            coords = list(map(float, parts[1:]))
            nodes_coords.append(coords)
            nodes[node_id] = c
            c += 1
    np_coords = np.array(nodes_coords)
    print(np_coords.shape)
    coords_h = np.hstack((np_coords, np.ones((np_coords.shape[0], 1))))
    voxel_index_t = (coords_h @ M_inv.T)[:, :3]
    voxel_index = np.floor(voxel_index_t).astype(int)
    print(voxel_index.shape)

    edges = set()
    with open(edge_path, "r") as f:
        for line in f:
            v1, v2 = tuple(map(int, line.strip().split(",")))
            v1_idx = nodes[v1]
            v2_idx = nodes[v2]
            if (v2_idx, v1_idx) not in edges:
                edges.add((v1_idx, v2_idx))

    violations = set()
    for v1,v2 in edges:
        coords_1 = voxel_index[v1]
        coords_2 = voxel_index[v2]
        # compute line voxels between v_pos_1 and v_pos_2
        line_voxels = np.array(line_nd(tuple(map(int, coords_1)), tuple(map(int, coords_2)))).T[1:] # skip the first entry, hence its the node itself
        for voxel in line_voxels:
            if voxel_grid[tuple(voxel)] == 1:
                violations.add((v1, v2))
                break

    print(f"Total outlier nodes: {len(violations)}")
    
process_run(run_path)




