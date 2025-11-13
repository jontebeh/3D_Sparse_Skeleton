import re
import pandas as pd
import json
from pathlib import Path
import numpy as np


evaluation_path = Path("./evaluation")
evaluation_path.mkdir(parents=True, exist_ok=True)
evaluation_file_path = evaluation_path / f"evaluation_area_safasdfasdfasdf.xlsx"



def load_shortest_paths(path: str) -> dict:
    # load path lengths
    shortest_path_results = Path(path + "shortest_paths.json")
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

def load_graph_stats(path: str) -> dict:
    result = {}
    graph_stats_path = Path(path + "graph_stats.json")
    if graph_stats_path.exists():
        with open(graph_stats_path, "r") as f:
            graph_stats = json.load(f)
        result.update(graph_stats)
    return result

def load_coverage_stats(path: str, voxel_size) -> dict:
    result = {}
    dist_map_path = Path(path + "dist.npy")
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

def load_stats(path: str) -> dict:
    result = {}
    stats_path = Path(path + "stats.json")
    if stats_path.exists():
        with open(stats_path, "r") as f:
            stats = json.load(f)
        result.update(stats)
    return result

def process_base(base: str) -> dict:
    result = {}

    # extract the area number
    result['area'] = int(base[base.find("area_")+5])
    size = float(re.search(r"size_0_(\d+)", base).group(1)) / 100.0
    result['voxel_size'] = size

    result['skeleton'] = "skeleton" in base

    print(f"Processing base: {base}")

    result.update(load_shortest_paths(base))

    transform_path = Path(base + "M.npy") if not result['skeleton'] else Path(base[:-9] + "M.npy")
    print(f"Loading transform from: {transform_path}")
    M = np.load(transform_path)
    voxel_size = M[0,0]

    result.update(load_graph_stats(base))
    result.update(load_stats(base))
    result.update(load_coverage_stats(base, voxel_size))

    return result


paths = [
    "./data/voxel_maps/area_1/area_1_size_0_05_skeleton_",
    "./data/voxel_maps/area_1/area_1_size_0_1_skeleton_",
    "./data/voxel_maps/area_1/area_1_size_0_15_skeleton_",
    "./data/voxel_maps/area_1/area_1_size_0_2_skeleton_",
    "./data/voxel_maps/area_2/Area_2_size_0_2_skeleton_",
    "./data/voxel_maps/area_3/area_3_size_0_2_skeleton_",
    "./data/voxel_maps/area_4/area_4_size_0_2_skeleton_",
    "./data/voxel_maps/area_5/area_5_size_0_2_skeleton_",
    "./data/voxel_maps/area_6/area_6_size_0_2_skeleton_",
    "./data/voxel_maps/area_1/area_1_size_0_05_",
    "./data/voxel_maps/area_1/area_1_size_0_1_",
    "./data/voxel_maps/area_1/area_1_size_0_15_",
    "./data/voxel_maps/area_1/area_1_size_0_2_",
    "./data/voxel_maps/area_2/Area_2_size_0_1_",
    "./data/voxel_maps/area_3/Area_3_size_0_1_",
    "./data/voxel_maps/area_4/Area_4_size_0_1_",
    "./data/voxel_maps/area_5/Area_5_size_0_1_",
    "./data/voxel_maps/area_6/Area_6_size_0_1_"
]

for base in paths:
    eval_result = process_base(base)
    print(eval_result)