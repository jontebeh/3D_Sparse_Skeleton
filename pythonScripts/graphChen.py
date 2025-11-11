from graph_tool import Graph
from graph_tool.topology import shortest_path
from pathlib import Path
import numpy as np
import timeit
import json

# load point pairs
area_path = Path("./data/voxel_maps/area_1/")
point_pairs = np.load(area_path / "area_1_size_0_1_point_pairs_voxel_indices.npy")
M = np.load(area_path / "area_1_size_0_1_M.npy")

voxel_grid = np.load(area_path / "area_1_size_0_1_voxel_grid.npy")

#run_path = Path("./output/tests/area_1_parameters/height/run_1762345333")
results_path = Path("./output/tests/area_1_down_sampling/")

voxel_size = M[0,0]

replace_files = True

def process_run(run: Path, point_pairs: np.ndarray):
    # load dependencies
    node_list_path = run / "node_list.txt"
    edge_list_path = run / "edge_list.txt"
    dist_map_path = run / "chen_dist_map.npy"
    id_map_full_path = run / "chen_id_map_full.npy"
    remap_path = run / "node_id_remap.npy"
    shortest_path_output = run / "shortest_path"
    graph_path = run / "graph.gt"

    # create dir if not exists
    shortest_path_output.mkdir(parents=True, exist_ok=True)

    node_remap = np.load(remap_path)
    remap_dict = dict(zip(node_remap[0], node_remap[1]))
    id_map_full = np.load(id_map_full_path)
    dist_map = np.load(dist_map_path)

    # check if the graph already exists
    if graph_path.exists() and not replace_files: # load graph
        G = Graph()
        G.load(str(graph_path))

        id_vertex_map = {}
        for v in G.vertices():
            node_id = G.vp['node_id'][v]
            id_vertex_map[node_id] = v
    else: # build graph
        G = Graph(directed=False)

        # load nodes
        id_vertex_map = {}
        vertex_coords = G.new_vertex_property("vector<double>")
        vertex_ids = G.new_vertex_property("int")
        with open(node_list_path, 'r') as f:
            for line in f:
                # use , as delimiter
                parts = line.strip().split(',')
                node_id = int(parts[0])
                x, y, z = map(float, parts[1:4])

                v = G.add_vertex()

                node_id = int(remap_dict.get(node_id, node_id))
                id_vertex_map[node_id] = v
                vertex_coords[v] = [x, y, z]
                vertex_ids[v] = node_id
        G.vp['coords'] = vertex_coords
        G.vp['node_id'] = vertex_ids

        weights = G.new_edge_property("double")
        with open(edge_list_path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                node_id_1 = int(parts[0])
                node_id_2 = int(parts[1])

                node_id_1 = int(remap_dict.get(node_id_1, node_id_1))
                node_id_2 = int(remap_dict.get(node_id_2, node_id_2))

                v1 = id_vertex_map[node_id_1]
                v2 = id_vertex_map[node_id_2]

                if not G.edge(v1,v2):
                    weight = np.linalg.norm(np.array(vertex_coords[v1]) - np.array(vertex_coords[v2]))
                    e = G.add_edge(v1, v2)
                    weights[e] = weight
        G.ep['weight'] = weights

        # save graph
        G.save(str(graph_path))

    # check if the shortest path already exists
    shortest_paths = shortest_path_output / "results.json"
    if shortest_paths.exists() and not replace_files: #skip computation
        print("Shortest path results already exist, skipping computation.")
    else: #compute shortest paths
        results = []
        for i in range(point_pairs.shape[0]):
            result = {}
            start_index = point_pairs[i, :3]
            end_index = point_pairs[i, 3:]
            result['start_index'] = tuple(map(int, start_index))
            result['end_index'] = tuple(map(int, end_index))

            print(f"Computing shortest path {i+1}/{point_pairs.shape[0]}: Start index: {start_index}, End index: {end_index}")

            start_id = id_map_full[tuple(map(int, start_index))]
            end_id = id_map_full[tuple(map(int, end_index))]
            result['start_id'] = int(start_id)
            result['end_id'] = int(end_id)

            start_vertex = id_vertex_map[start_id]
            end_vertex = id_vertex_map[end_id]

            start_time = timeit.default_timer()
            path, edges = shortest_path(G, start_vertex, end_vertex, weights=G.ep['weight'])
            end_time = timeit.default_timer()
            print(f"Shortest path computed in {end_time - start_time:.4f} seconds.")
            result['computation_time'] = end_time - start_time
            
            result['path'] = [G.vp['node_id'][v] for v in path]
            result['length'] = sum([G.ep['weight'][e] for e in edges])
            result['total_length'] = result['length'] + dist_map[tuple(start_index)] * voxel_size + dist_map[tuple(end_index)] * voxel_size
            results.append(result)

        # save results
        with open(shortest_paths, 'w') as f:
            json.dump(results, f, indent=4, sort_keys=True)
        print(f"Shortest path results saved to {shortest_paths}.")
    
    # check if graph_stats.json exists
    graph_stats_path = run / "graph_stats.json"
    if graph_stats_path.exists() and not replace_files: # skip computation
        print("Graph stats already exist, skipping computation.")
    else: # compute graph stats
        num_vertices = G.num_vertices()
        num_edges = G.num_edges()
        degrees = [v.out_degree() for v in G.vertices()]
        avg_degree = sum(degrees) / num_vertices
        avg_edge_length = sum(G.ep['weight'][e] for e in G.edges()) / num_edges
        num_vertices_deg_greater_2 = sum(1 for v in G.vertices() if v.out_degree() > 2)
        dead_ends = sum(1 for v in G.vertices() if v.out_degree() == 1)


        graph_stats = {
            'num_vertices': num_vertices,
            'num_edges': num_edges,
            'avg_degree': avg_degree,
            'avg_edge_length': avg_edge_length,
            'num_vertices_deg_greater_2': num_vertices_deg_greater_2,
            'num_dead_ends': dead_ends
        }
        with open(graph_stats_path, 'w') as f:
            json.dump(graph_stats, f, indent=4, sort_keys=True)
        print(f"Graph stats saved to {graph_stats_path}.")

def recursive_process_runs(base_path: Path) -> bool:
    if "run_" in base_path.name:
        try:
            return process_run(base_path, point_pairs)
        except Exception as e:
            print(f"Error processing {base_path}: {e}")
            return False
    sub_dirs = [d for d in base_path.iterdir() if d.is_dir()]
    if not sub_dirs:
        return False
    
    processed_any = False
    for sub_dir in sub_dirs:
        processed = recursive_process_runs(sub_dir)
        processed_any = processed_any or processed
    return processed_any

print(f"Processing runs in {results_path}...")
print(f"Results found: {recursive_process_runs(results_path)}")