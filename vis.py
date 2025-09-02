# Visualizer for a point cloud dataset
import open3d as o3d


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


def main():
    # Replace with your point cloud file path
    file_path = "./modular_polygon_generation/libcore/data/maps/area_1.pcd"

    node_path = "./build/node_list.txt"
    edge_path = "./build/edge_list.txt"

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
    pcd = load_point_cloud(file_path)

    # Visualize the point cloud
    visualize_point_cloud(pcd, nodes, edges)

if __name__ == "__main__":
    main()