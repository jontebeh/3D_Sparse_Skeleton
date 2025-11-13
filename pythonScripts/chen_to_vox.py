from functions import *
from pathlib import Path

VISUALIZE = False

area_nr = 6

data_path = Path(f"./data/voxel_maps/area_{area_nr}/")
voxel_grid = np.load(data_path / f"Area_{area_nr}_size_0_1_voxel_grid.npy")
voxel_grid_flooded = np.load(data_path / f"Area_{area_nr}_size_0_1_voxel_grid_flooded.npy")
M = np.load(data_path / f"Area_{area_nr}_size_0_1_M.npy")

results_path = Path("./output/tests/areas/run_1763047891/")

def recursive_process_runs(base_path: Path) -> bool:
    if "run_" in base_path.name:
        return process_run(base_path)
    sub_dirs = [d for d in base_path.iterdir() if d.is_dir()]
    if not sub_dirs:
        return False
    
    processed_any = False
    for sub_dir in sub_dirs:
        processed = recursive_process_runs(sub_dir)
        processed_any = processed_any or processed
    return processed_any
        
def process_run(run: Path) -> bool:
    if not (run / "node_list.txt").exists() or not (run / "edge_list.txt").exists():
        return False

    G, remap_node_ids = load_graph(run / "node_list.txt", run / "edge_list.txt")
    run_voxel, run_voxel_id_map = get_voxel_from_graph(G, M, voxel_grid.shape)

    if VISUALIZE:
        visualize([VisObjGraph(G), VisObjVoxel(voxel_grid_flooded, M), VisObjVoxel(run_voxel, M)])
    
    if not (run / "chen_voxel.npy").exists():
        np.save(run / "chen_voxel.npy", run_voxel)
        print(f"Saved voxel grid with shape {run_voxel.shape} to {run / 'chen_voxel.npy'}")
    
    if not (run / "chen_voxel_id_map.npy").exists():
        np.save(run / "chen_voxel_id_map.npy", run_voxel_id_map)
        print(f"Saved voxel ID map with shape {run_voxel_id_map.shape} to {run / 'chen_voxel_id_map.npy'}")
    
    if not (run / "node_id_remap.npy").exists():
        # convert remap_node_ids to array 2xN
        old_ids = np.array(list(remap_node_ids.keys()))
        new_ids = np.array(list(remap_node_ids.values()))
        remap_array = np.vstack((old_ids, new_ids))
        np.save(run / "node_id_remap.npy", remap_array)
        print(f"Saved node ID remap with shape {remap_array.shape} to {run / 'node_id_remap.npy'}")

    return True

print(f"Processing runs in {results_path}...")
print(f"Results found: {recursive_process_runs(results_path)}")