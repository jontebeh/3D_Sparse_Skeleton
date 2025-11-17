# 3D Sparse Skeleton – Bachelor Thesis Implementation

This repository contains the code used in my Bachelor's thesis on 3D sparse topological skeleton graphs for indoor mobile-robot path planning.  
It implements and extends the algorithm from:

> Xinyi Chen et al., **"Fast 3D Sparse Topological Skeleton Graph Generation for Mobile Robot Global Planning" (IROS 2022)**

to work as a modular C++ pipeline for offline skeletonization and evaluation on large 3D indoor datasets (e.g., S3DIS), including extensive parameter studies and analysis scripts.

## Origin / Reference

This work is based on and extends the original implementation by Chen et al.:  
https://github.com/xchencq/3D_Sparse_Skeleton

## Repository Structure

- `modular_polygon_generation/` – main C++ implementation of the 3D skeletonization pipeline  
- `pythonScripts/` – Python/Jupyter utilities for preprocessing, analysis and plotting  
- `evaluation/` – results of the evaluation experiments
- `2D_maps/` – 2D maps generated from 3D skeletons and other output data
- `MSSP/` – C++ Implementation for a coverage map
- `output/` – output data from the skeletonization runs

## Build

The project uses CMake:

```bash
git clone https://github.com/jontebeh/3D_Sparse_Skeleton.git
cd 3D_Sparse_Skeleton
mkdir build && cd build
cmake ..
make -j
```

## Usage
The main executable is `run_skeleton`. A configuration file defined in the `main.cpp`
is required to run the skeletonization pipeline. There are multiple example configuration files available in the `config/` folder.
Also provoide a map file in PCD format in the `config/` folder as well.
