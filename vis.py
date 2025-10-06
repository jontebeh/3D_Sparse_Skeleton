# Visualizer for a point cloud dataset
import open3d as o3d
from pathlib import Path

def load_point_cloud(file_path):
    """Loads a point cloud from a file."""
    try:
        pcd = o3d.io.read_point_cloud(file_path)
        if not pcd.has_points():
            raise ValueError("Point cloud has no points.")
        return pcd
    except Exception as e:
        print(f"Error loading point cloud from {file_path}: {e}")
        return None

def visualize_point_cloud(pcd, nodes, edges):
    """Visualizes a point cloud with red spheres and green edge lines."""
    if pcd is None:
        print("No point cloud to visualize.")
        return

    # Create red spheres at node positions
    spheres = []
    node_positions = [node[1:] for node in nodes]  # Extract 3D positions

    for pos in node_positions:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red
        sphere.translate(pos)
        spheres.append(sphere)

    # Create green lines between nodes
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(node_positions)  # Points for indexing
    line_set.lines = o3d.utility.Vector2iVector(edges)  # Edges as index pairs
    line_set.paint_uniform_color([0.0, 1.0, 0.0])  # Green lines

    # Visualize
    o3d.visualization.draw_geometries(
        [pcd, line_set] + spheres,
        window_name="Point Cloud with Nodes and Edges",
        width=1600,
        height=900
    )

def getRunPath(output_dir: Path, run: str) -> Path:
    if run == "latest":
        # get all directories in output_dir
        runs = [d for d in output_dir.iterdir() if d.is_dir()]
        # sort by time in run_time name
        runs = sorted(runs, key=lambda x: x.stat().st_mtime, reverse=True)
        if len(runs) == 0:
            raise ValueError("No runs found in output directory.")
        print(f"Using latest run: {runs[0]}")
        return runs[0]
    else:
        if "run_" not in run:
            run = "run_" + run
        run_path = output_dir / run
        if not run_path.exists():
            raise ValueError(f"Run {run} does not exist in output directory.")
        print(f"Using specified run: {run_path}")
        return run_path

def getMapPath(run_path: Path) -> Path:
    # read the ini file in the run_path
    ini_files = list(run_path.glob("*.ini"))
    if len(ini_files) == 0:
        raise ValueError(f"No ini file found in run path {run_path}.")
    ini_file = ini_files[0]
    # read the map line
    map_file = None
    vis_map_file = None
    with open(ini_file, 'r') as f:
        for line in f:
            line = line.strip()
            # remove tabs and spaces
            line = ''.join(line.split())
            if line.startswith("map_name="):
                map_file = line.split('=')[1].strip()
            if line.startswith("vis_map_name="):
                vis_map_file = line.split('=')[1].strip()
    if vis_map_file:
        return Path("./modular_polygon_generation/libcore/data/maps/") / vis_map_file
    elif map_file:
        return Path("./modular_polygon_generation/libcore/data/maps/") / map_file
    else:
        raise ValueError(f"No map_name or vis_map_name found in ini file {ini_file}.")

def main():
    output_dir = Path("./output/")
    run = "1756996692"

    run_path = getRunPath(output_dir, run)
    node_path = run_path / "node_list.txt"
    edge_path = run_path / "edge_list.txt"
    map_path = getMapPath(run_path)

    print(f"Using run path: {run_path}")
    print(f"Map: {map_path}")

    # read the node list with , separators
    nodes = []
    with open(node_path, 'r') as f:
        for line in f:
            if line.strip():  # check if the line is not empty
                nodes.append(line.strip().split(','))

    # convert to float
    nodes = [[float(coord) for coord in node] for node in nodes]

    edges = []
    with open(edge_path, 'r') as f:
        for line in f:
            if line.strip():  # check if the line is not empty
                edges.append(line.strip().split(','))
    # convert to int
    edges = [[int(edge[0]), int(edge[1])] for edge in edges]


    # Load the point cloud
    pcd = load_point_cloud(map_path)

    # Visualize the point cloud
    visualize_point_cloud(pcd, nodes, edges)

if __name__ == "__main__":
    main()