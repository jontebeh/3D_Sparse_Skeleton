# Projection of 3D points and edges onto a 2D plane
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from skimage import io, morphology, measure


def getRunPath(output_dir: Path, run: str) -> Path:
    if run == "latest":
        # get all directories in output_dir
        runs = [d for d in output_dir.iterdir() if d.is_dir()]
        # sort by time in run_time name
        runs = sorted(runs, key=lambda x: x.stat().st_mtime, reverse=True)
        if len(runs) == 0:
            raise ValueError("No runs found in output directory.")
        return runs[0]
    else:
        if "run_" not in run:
            run = "run_" + run
        run_path = output_dir / run
        if not run_path.exists():
            raise ValueError(f"Run {run} does not exist in output directory.")
        return run_path

def getMapPath(run_path: Path) -> tuple[Path, Path]:
    # read the ini file in the run_path
    ini_files = list(run_path.glob("*.ini"))
    if len(ini_files) == 0:
        raise ValueError(f"No ini file found in run path {run_path}.")
    ini_file = ini_files[0]
    # read the map line
    map_file = None
    with open(ini_file, 'r') as f:
        for line in f:
            # remove all whitespace and spaces
            line_strip = line.strip()
            line_strip = line_strip.replace(" ", "")
            if line_strip.startswith("map_name="):
                map_file = line_strip.split('=')[1].strip()
    if not map_file:
        raise ValueError(f"No map_name found in ini file {ini_file}.")
    
    map_name = Path(map_file).stem
    print(f"Found map name: {map_name}")

    # find map in 2D_maps
    maps_2d = Path("2D_maps")

    # find a file that contains map_name in lowercase
    map_files = [f for f in maps_2d.iterdir() if map_name.lower() in f.name.lower()]
    if len(map_files) == 0:
        raise ValueError(f"No map file found for map name {map_name} in 2D_maps.")

    # check if there is a meta ply file and a non-meta png file
    meta_file = None
    non_meta_file = None
    for f in map_files:
        if "meta" in f.name.lower():
            meta_file = f
        else:
            non_meta_file = f

    if not non_meta_file:
        raise ValueError(f"No non-meta map file found for map name {map_name} in 2D_maps.")
    if not meta_file:
        raise ValueError(f"No meta map file found for map name {map_name} in 2D_maps.")

    return non_meta_file, meta_file

def getNodesAndEdges(run_path: Path) -> tuple[np.ndarray, np.ndarray]:
    node_path = run_path / "node_list.txt"
    edge_path = run_path / "edge_list.txt"

    # read the node list with , separators
    nodes = []
    node_ids = []
    with open(node_path, 'r') as f:
        for line in f:
            if line.strip():  # check if the line is not empty
                node = line.strip().split(',')
                nodes.append(node[1:4])  # skip the first column
                node_ids.append(int(node[0]))  # keep the first column as id

    # convert to float np array
    nodes = np.array(nodes, dtype=float)
    node_ids = np.array(node_ids, dtype=int)

    edges = []
    with open(edge_path, 'r') as f:
        for line in f:
            if line.strip():  # check if the line is not empty
                edges.append(line.strip().split(','))
    # convert to int array
    edges = [[int(edge[0]), int(edge[1])] for edge in edges]

    # convert edge ids to corresponding indices in nodes
    for i, edge in enumerate(edges):
        # get id 1 and id 2
        id1 = edge[0]
        id2 = edge[1]

        # get the id from node_ids
        id1_index = np.where(node_ids == id1)[0]
        id2_index = np.where(node_ids == id2)[0]

        if len(id1_index) == 0 or len(id2_index) == 0:
            print(f"Edge with non-existing node id: {id1}, {id2}")
            continue
        if id1_index[0] == id2_index[0]:
            print(f"Edge with same node id: {id1}, {id2}")
            continue

        # update edges to use indices
        edges[i][0] = id1_index[0]
        edges[i][1] = id2_index[0]

    return nodes, edges

def plotAll(img: np.ndarray, nodes: np.ndarray, edges: np.ndarray):
    # flip the img horizontally
    img_cp = img.copy()
    img_cp = np.flipud(img_cp)

    # flip nodes and edges horizontally
    nodes_flipped = nodes.copy()
    # flip y coordinate
    nodes_flipped[:, 1] = img_cp.shape[0] - nodes_flipped[:, 1]


    plt.imshow(img_cp)
    plt.scatter(nodes_flipped[:, 0], nodes_flipped[:, 1], c='red')

    for edge in edges:
        node1 = nodes_flipped[edge[0]]
        node2 = nodes_flipped[edge[1]]
        plt.plot([node1[0], node2[0]], [node1[1], node2[1]], c='blue')
    plt.show()


def main():
    output_dir = Path("./output/")
    run = "1756996692"

    run_path = getRunPath(output_dir, run)
    map_path, meta_path = getMapPath(run_path)

    print(f"Using run path: {run_path}")
    print(f"Map: {map_path}")
    print(f"Meta: {meta_path}")

    nodes, edges = getNodesAndEdges(run_path)
    print(f"Loaded {len(nodes)} nodes and {len(edges)} edges.")

    # load the map image as np array
    map_img = plt.imread(map_path)

    # print the shape of the map
    print(f"Map shape: {map_img.shape}")

    map_binary = map_img[:, :, 0]  # assume grayscale or use red channel

    occ = map_binary < 0.5  # assume white=obstacle
    free = ~occ

    # Compute medial axis skeleton
    vorn = morphology.medial_axis(free)

    skeleton = morphology.skeletonize(free)

    map_img[vorn] = [0, 1, 0, 1]  # green for voronoi
    map_img[skeleton] = [1, 0, 0, 1]  # red for skeleton

    # load the transformation matrix from meta ply file
    M = np.load(meta_path)
    print(f"Transformation matrix:\n{M}")

    # remove z coordinate from nodes
    nodes_2d = nodes[:, :2]

    # ranges
    print(f"Node ranges x: {np.min(nodes_2d[:,0])} - {np.max(nodes_2d[:,0])}")
    print(f"Node ranges y: {np.min(nodes_2d[:,1])} - {np.max(nodes_2d[:,1])}")

    # add ones to nodes for homogeneous coordinates
    ones = np.ones((nodes_2d.shape[0], 1))
    nodes_homogeneous = np.hstack((nodes_2d, ones))

    # apply transformation matrix
    nodes_transformed = (M @ nodes_homogeneous.T).T

    # before 

    # plot the map and the nodes and edges
    plotAll(map_img, nodes_transformed[:, :2], edges)



if __name__ == "__main__":
    main()