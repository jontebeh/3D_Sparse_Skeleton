from graph_tool import Graph
from graph_tool.topology import shortest_path
from pathlib import Path
import sys
import time
import numpy as np
import timeit
import json

area_nr = 1
size = "0_05"

# load point pairs
area_path = Path(f"./data/voxel_maps/area_{area_nr}/")
point_pairs_world = np.load(area_path / f"area_{area_nr}_size_0_1_point_pairs_world_coords.npy")

graph_path = area_path / f"area_{area_nr}_size_{size}_skeleton_graph.gt"
dist_map_path = area_path / f"area_{area_nr}_size_{size}_skeleton_dist.npy"
id_map_full_path = area_path / f"area_{area_nr}_size_{size}_skeleton_ids_full.npy"
shortest_path_output = area_path / f"area_{area_nr}_size_{size}_skeleton_shortest_paths.json"

M = np.load(area_path / f"area_{area_nr}_size_{size}_M.npy")
id_map = np.load(id_map_full_path)
dist_map = np.load(dist_map_path)

voxel_grid = np.load(area_path / f"area_{area_nr}_size_{size}_skeleton.npy")
voxel_size = M[0,0]

# compute point_pairs based on M
# split into start and end points
start_points_world = point_pairs_world[:, :3]
end_points_world = point_pairs_world[:, 3:]
# convert to voxel coordinates
start_points_world_h = np.hstack((start_points_world, np.ones((start_points_world.shape[0], 1))))
end_points_world_h = np.hstack((end_points_world, np.ones((end_points_world.shape[0], 1))))
M_inv = np.linalg.inv(M)
start_points_voxel_h = (M_inv @ start_points_world_h.T)[:3, :].T
end_points_voxel_h = (M_inv @ end_points_world_h.T)[:3, :].T
# floor to get voxel indices
start_points_voxel = np.floor(start_points_voxel_h).astype(int)
end_points_voxel = np.floor(end_points_voxel_h).astype(int)
# combine back to point pairs
point_pairs = np.hstack((start_points_voxel, end_points_voxel))

def progress_bar(progress, total, length=50):
    percent = int(progress / total * 100)
    percent_detailed = progress / total * 100
    filled = int(length * progress / total)
    bar = '#' * filled + '-' * (length - filled)
    sys.stdout.write(f"\r|{bar}| {percent_detailed:.3f}%")
    sys.stdout.flush()

def process_voxel(voxel_grid: np.ndarray, point_pairs: np.ndarray):
    if graph_path.exists(): # load graph
        print(f"Loading existing graph from file {graph_path}")
        G = Graph()
        G.load(str(graph_path))

        id_vertex_map = {}
        for i, v in enumerate(G.vertices()):
            if i % 100 == 0:
                progress_bar(i, G.num_vertices())
            node_id = int(G.vp['node_id'][v])
            id_vertex_map[node_id] = v
        progress_bar(1, 1)
        print()

        print(f"Graph loaded with vertex property: {list(G.vp.keys())} and edge property: {list(G.ep.keys())}")

        # print a few ids for debugging
        sample_ids = list(id_vertex_map.keys())[:5]
        print(f"Sample node IDs in the graph: {sample_ids}")
    else: # build graph
        print("Building graph from voxel grid...")
        id_vertex_map = {}
        G = Graph(directed=False)
        print("Adding vertices...")
        indices = np.argwhere(voxel_grid == 1)
        print(f"Total vertices to add: {len(indices)}")
        vertex_ids = G.new_vp("int")
        for i, idx in enumerate(indices):
            if i % 100 == 0:
                progress_bar(i, len(indices))
            v = G.add_vertex()
            node_id = int(id_map[tuple(idx)])
            if node_id == 0:
                raise ValueError(f"Voxel at index {idx} has invalid node ID 0 in id_map_full.")
            id_vertex_map[node_id] = v
            vertex_ids[v] = node_id
        G.vp['node_id'] = vertex_ids
        progress_bar(1, 1)
        print()

        print("Adding edges...")
        n26_offsets = np.array([[i, j, k] for i in [-1, 0, 1] for j in [-1, 0, 1] for k in [-1, 0, 1] if not (i == 0 and j == 0 and k == 0)])
        n26_weights = np.linalg.norm(n26_offsets, axis=1) * voxel_size

        weights = G.new_ep("double")
        for i, idx in enumerate(indices):
            if i % 100 == 0:
                progress_bar(i, len(indices))
            node_id_1 = int(id_map[tuple(idx)])
            v1 = id_vertex_map[node_id_1]
            for offset, weight in zip(n26_offsets, n26_weights):
                neighbor_idx = idx + offset
                neighbor_id = int(id_map[tuple(neighbor_idx)])
                if (0 <= neighbor_idx[0] < voxel_grid.shape[0] and
                    0 <= neighbor_idx[1] < voxel_grid.shape[1] and
                    0 <= neighbor_idx[2] < voxel_grid.shape[2] and
                    voxel_grid[tuple(neighbor_idx)] == 1):
                    v2 = id_vertex_map[neighbor_id]
                    # check if edge already exists
                    if not G.edge(v1, v2):
                        e = G.add_edge(v1, v2)
                        weights[e] = weight
        G.ep['weight'] = weights
        progress_bar(1, 1)
        print()

        # save graph
        G.save(str(graph_path))
        print(f"Graph saved to {graph_path}.")

    # check if the shortest path already exists
    if shortest_path_output.exists(): #skip computation
        print("Shortest path results already exist, skipping computation.")
    else: #compute shortest paths
        print("Computing shortest paths...")
        results = []
        for i in range(point_pairs.shape[0]):
            result = {}
            start_id = int(id_map[tuple(point_pairs[i, :3])])
            end_id = int(id_map[tuple(point_pairs[i, 3:])])
            result['start_id'] = start_id
            result['end_id'] = end_id

            if start_id == 0 or end_id == 0:
                print(f"Warning: Start ID {start_id} or End ID {end_id} is invalid (0). Skipping this pair.")
                continue

            print(f"Computing shortest path {i+1}/{point_pairs.shape[0]}: Start index: {start_id}, End index: {end_id}")

            start_vertex = id_vertex_map[start_id]
            end_vertex = id_vertex_map[end_id]

            start_time = timeit.default_timer()
            path, edges = shortest_path(G, start_vertex, end_vertex, weights=G.ep['weight'])
            end_time = timeit.default_timer()
            print(f"Shortest path computed in {end_time - start_time:.4f} seconds.")
            result['computation_time'] = end_time - start_time

            result['path'] = [int(G.vp['node_id'][v]) for v in path]
            result['length'] = sum([G.ep['weight'][e] for e in edges])
            result['total_length'] = result['length'] + dist_map[tuple(point_pairs[i, :3])] * voxel_size + dist_map[tuple(point_pairs[i, 3:])] * voxel_size
            results.append(result)

        # save results
        with open(shortest_path_output, 'w') as f:
            json.dump(results, f, indent=4, sort_keys=True)
        print(f"Shortest path results saved to {shortest_path_output}.")
    
    # check if graph_stats.json exists
    graph_stats_path = area_path / f"Area_{area_nr}_size_{size}_skeleton_graph_stats.json"
    if graph_stats_path.exists():
        print("Graph stats already exist, skipping computation.")
    else: # compute graph stats
        print("Computing graph statistics...")
        num_vertices = G.num_vertices()
        num_edges = G.num_edges()
        avg_degree = 2 * num_edges / num_vertices


        graph_stats = {
            'num_vertices': num_vertices,
            'num_edges': num_edges,
            'avg_degree': avg_degree
        }
        with open(graph_stats_path, 'w') as f:
            json.dump(graph_stats, f, indent=4, sort_keys=True)
        print(f"Graph stats saved to {graph_stats_path}.")

process_voxel(voxel_grid, point_pairs)