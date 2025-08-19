cmake_minimum_required(VERSION 3.10)
project(modular_polygon_generation)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io search)
set(Open3D_DIR "$ENV{HOME}/libs/open3d-install/lib/cmake/Open3D")
find_package(Open3D REQUIRED COMPONENTS Gui)
# Core library
add_library(libcore
  modular_polygon_generation/libcore/src/skeleton_finder.cpp
  modular_polygon_generation/libcore/src/visualizer.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/map_interface.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/expansion.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/facet_utils.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/frontier_utils.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/geometry_utils.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/raycasting.cpp
  modular_polygon_generation/libcore/src/skeleton_utils/sampling.cpp
  modular_polygon_generation/libcore/include/libcore/quickhull/QuickHull.cpp
  
  # Add other source files if needed
)

target_include_directories(libcore PUBLIC
  modular_polygon_generation/libcore/include
  modular_polygon_generation/libcore/include/libcore/quickhull
  modular_polygon_generation/libcore/include/libcore/quickhull/Structs
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

target_link_libraries(libcore
  ${PCL_LIBRARIES}
  Open3D::Open3D
)

# CLI executable
add_executable(run_skeleton modular_polygon_generation/libcore/src/main.cpp)
target_link_libraries(run_skeleton libcore)

# Optional: Tests
enable_testing()
#add_subdirectory(tests)

# for debugging
#set(CMAKE_BUILD_TYPE Debug)
