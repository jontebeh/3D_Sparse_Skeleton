from graph_tool import Graph
from graph_tool.draw import graph_draw
from graph_tool.search import astar_search
from graph_tool.topology import shortest_path
from pathlib import Path
import sys
import time
import numpy as np
import timeit
import json

# load point pairs
area_path = Path("./data/voxel_maps/area_1/")
point_pairs = np.load(area_path / "area_1_size_0_1_point_pairs_voxel_indices.npy")
graph_path = area_path / "area_1_size_0_1_graph.gt"

M = np.load(area_path / "area_1_size_0_1_M.npy")

voxel_grid = np.load(area_path / "area_1_size_0_1_voxel_grid_inverted.npy")

voxel_size = M[0,0]


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
            node_id = G.vp['node_id'][v]
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
        vertex_ids = G.new_vp("vector<int64_t>")
        for i, idx in enumerate(indices):
            if i % 100 == 0:
                progress_bar(i, len(indices))
            v = G.add_vertex()
            node_id = tuple(map(int, idx))
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
            node_id_1 = tuple(map(int, idx))
            v1 = id_vertex_map[node_id_1]
            for offset, weight in zip(n26_offsets, n26_weights):
                neighbor_idx = idx + offset
                neighbor_id = tuple(map(int, neighbor_idx))
                if (0 <= neighbor_idx[0] < voxel_grid.shape[0] and
                    0 <= neighbor_idx[1] < voxel_grid.shape[1] and
                    0 <= neighbor_idx[2] < voxel_grid.shape[2] and
                    voxel_grid[neighbor_id] == 1):
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
    shortest_paths = area_path / "area_1_size_0_1_shortest_paths.json"
    if shortest_paths.exists(): #skip computation
        print("Shortest path results already exist, skipping computation.")
    else: #compute shortest paths
        results = []
        for i in range(point_pairs.shape[0]):
            result = {}
            start_id = tuple(map(int, point_pairs[i, :3]))
            end_id = tuple(map(int, point_pairs[i, 3:]))
            result['start_id'] = start_id
            result['end_id'] = end_id

            print(f"Computing shortest path {i+1}/{point_pairs.shape[0]}: Start index: {start_id}, End index: {end_id}")

            start_vertex = id_vertex_map[start_id]
            end_vertex = id_vertex_map[end_id]

            start_time = timeit.default_timer()
            path, edges = shortest_path(G, start_vertex, end_vertex, weights=G.ep['weight'])
            end_time = timeit.default_timer()
            print(f"Shortest path computed in {end_time - start_time:.4f} seconds.")
            result['computation_time'] = end_time - start_time

            result['path'] = [tuple(G.vp['node_id'][v]) for v in path]
            result['total_length'] = sum([G.ep['weight'][e] for e in edges])
            results.append(result)

        # save results
        with open(shortest_paths, 'w') as f:
            json.dump(results, f, indent=4, sort_keys=True)
        print(f"Shortest path results saved to {shortest_paths}.")
    
    # check if graph_stats.json exists
    graph_stats_path = area_path / "area_1_size_0_1_graph_stats.json"
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