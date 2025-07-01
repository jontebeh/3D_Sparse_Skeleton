cmake_minimum_required(VERSION 3.10)
project(modular_polygon_generation)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# Core library
add_library(libcore
  libcore/src/skeleton_finder.cpp
)

target_include_directories(libcore PUBLIC
  libcore/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(libcore
  ${PCL_LIBRARIES}           
)

  
# CLI executable
#add_executable(run_skeleton cli/run_skeleton.cpp)
#target_link_libraries(run_skeleton libcore)

# Optional: Tests
enable_testing()
#add_subdirectory(tests)
