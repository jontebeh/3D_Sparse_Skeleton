import pandas as pd
import json
from pathlib import Path
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

tests_path = Path("./output/tests/area_1_parameters")

area_path = Path("./data/voxel_maps/area_1")

evaluation_path = Path("./evaluation")
evaluation_path.mkdir(parents=True, exist_ok=True)

M = np.load(area_path / "area_1_size_0_1_M.npy")
voxel_size = M[0,0]

def load_shortest_paths(path: Path) -> dict:
    # load path lengths
    shortest_path_results = path / "results.json"
    result = {}
    if shortest_path_results.exists():
        with open(shortest_path_results, "r") as f:
            data = json.load(f)

        lengths = []
        times = []
        for experiment in data:
            lengths.append(experiment['total_length'])
            times.append(experiment['computation_time'])
        np_lengths = np.array(lengths)
        np_times = np.array(times)
        
        
        # add path lengths statistics
        result['mean_path_length'] = float(np.mean(np_lengths))
        result['std_path_length'] = float(np.std(np_lengths))
        # add max and min path lengths
        result['max_path_length'] = float(np.max(np_lengths))
        result['min_path_length'] = float(np.min(np_lengths))

        result['mean_path_computation_time'] = float(np.mean(np_times))
        result['std_path_computation_time'] = float(np.std(np_times))
        result['max_path_computation_time'] = float(np.max(np_times))
        result['min_path_computation_time'] = float(np.min(np_times))
    return result

def load_config(path: Path) -> dict:
    # parse a ini config to a dict
    config = {}
    with open(path, "r") as f:
        for line in f:
            if '=' in line:
                key, value = line.split('=', 1)
                config[key.strip()] = value.strip()
    return config

def load_time_logs(path: Path) -> dict:
    result = {}
    # get runtime from log
    log_path = next(path.glob("*.log"), None)
    if log_path:
        with open(log_path, "r") as f:
            start_time = None
            end_time = None
            for line in f:
                # extract the time from [hh:mm:ss] at the start of the line
                if "[Info] Generating skeleton..." in line:
                    start_time = datetime.strptime(line.split(']')[0].strip('['), "%H:%M:%S")
                if "[Info] Skeleton expansion completed." in line:
                    end_time = datetime.strptime(line.split(']')[0].strip('['), "%H:%M:%S")
            result['start_time'] = start_time
            result['end_time'] = end_time
            if start_time and end_time:
                result['total_time'] = (end_time - start_time).total_seconds()
    return result

def load_graph_stats(path: Path) -> dict:
    result = {}
    graph_stats_path = path / "graph_stats.json"
    if graph_stats_path.exists():
        with open(graph_stats_path, "r") as f:
            graph_stats = json.load(f)
        result.update(graph_stats)
    return result

def load_coverage_stats(path: Path) -> dict:
    result = {}
    dist_map_path = path / "chen_dist_map.npy"
    if dist_map_path.exists():
        dist_map = np.load(dist_map_path)
        # replace everything below 0 with 0
        dist_map[dist_map < 0] = 0
        # adjust to voxel size
        dist_map = dist_map * voxel_size
        # compute coverage stats
        result['mean_coverage_distance'] = float(np.mean(dist_map))
        result['std_coverage_distance'] = float(np.std(dist_map))
        result['max_coverage_distance'] = float(np.max(dist_map))
        pos = dist_map[dist_map > 0]
        result['min_coverage_distance'] = float(np.min(pos)) if pos.size > 0 else None

    return result

def load_node_stats(path: Path) -> dict:
    result = {}
    nodes_path = path / "node_list.txt"
    if nodes_path.exists():
        nodes = []
        with open(nodes_path, "r") as f:
            for line in f:
                parts = line.strip().split(',')
                nodes.append(list(map(float, parts[1:4])))
        
        if not nodes:
            return result
        
        np_nodes = np.array(nodes)
        result['avg_node_height'] = float(np.mean(np_nodes[:, 2]))
        result['std_node_height'] = float(np.std(np_nodes[:, 2]))
        result['max_node_height'] = float(np.max(np_nodes[:, 2]))
        result['min_node_height'] = float(np.min(np_nodes[:, 2]))
        # create a histogram of node heights vor every 0.2 meters
        hist = np.floor(np_nodes[:, 2] / 0.2) * 0.2
        unique, counts = np.unique(hist, return_counts=True)
        for u, c in zip(unique, counts):
            result[f'node_height_{u:.1f}_count'] = int(c) / len(np_nodes)

    return result

def process_run(run_path: Path) -> dict:
    result = {}
    # load ini (first ini file in the run folder)
    config_path = next(run_path.glob("*.ini"), None)

    run_id = run_path.name
    result['run_id'] = run_id

    run_name = config_path.stem
    result['run_name'] = run_name
    print(f"Processing run: {run_id} - {run_name}")

    result.update(load_config(config_path))

    shortest_paths_path = run_path / "shortest_path"
    result.update(load_shortest_paths(shortest_paths_path))

    result.update(load_graph_stats(run_path))
    result.update(load_time_logs(run_path))
    result.update(load_coverage_stats(run_path))
    result.update(load_node_stats(run_path))

    return result


pages = {}
for parameter_folder in tests_path.iterdir():
    if parameter_folder.is_dir():
        all_results = []
        for run_folder in parameter_folder.iterdir():
            if run_folder.is_dir() and run_folder.name.startswith("run_"):
                res = process_run(run_folder)
                all_results.append(res)
        # create dataframe
        df = pd.DataFrame(all_results)
        # save name for excel page
        page_name = parameter_folder.name
        pages[page_name] = df



# save to excel with multiple sheets
with pd.ExcelWriter(evaluation_path / "evaluation_area_1_parameters.xlsx") as writer:
    for page_name, df in pages.items():
        # truncate page name to 31 characters (excel limit)
        sheet_name = page_name[:31]
        df.to_excel(writer, sheet_name=sheet_name, index=False)