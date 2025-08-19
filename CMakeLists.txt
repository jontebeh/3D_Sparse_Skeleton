cmake_minimum_required(VERSION 3.10)
project(modular_polygon_generation)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io search)

# Tell CMake where your Open3D install is
set(Open3D_DIR "$ENV{HOME}/libs/open3d-gui-install/lib/cmake/Open3D")
find_package(Open3D REQUIRED COMPONENTS Gui)

# -------------------- libcore --------------------
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

# -------------------- run_skeleton exe --------------------
add_executable(run_skeleton modular_polygon_generation/libcore/src/main.cpp)
target_link_libraries(run_skeleton libcore)

# ---------- Copy Open3D GUI resources next to the binary (if we can find them) ----------
set(OPEN3D_RES_DIR "" CACHE PATH "Path to Open3D GUI resources")

if(NOT OPEN3D_RES_DIR)
  # Open3D_DIR is <prefix>/lib/cmake/Open3D
  get_filename_component(_open3d_prefix "${Open3D_DIR}/../../.." ABSOLUTE)

  set(_cands
    "${_open3d_prefix}/share/open3d/resources"
    "${_open3d_prefix}/share/Open3D/resources"
    "${_open3d_prefix}/share/resources"                # <— your install
    "${_open3d_prefix}/bin/Open3D/resources"           # <— your install
    "${_open3d_prefix}/resources"
    "${_open3d_prefix}/../share/resources"
  )

  foreach(_p IN LISTS _cands)
    if(EXISTS "${_p}")
      set(OPEN3D_RES_DIR "${_p}")
      break()
    endif()
  endforeach()
endif()

if(OPEN3D_RES_DIR AND EXISTS "${OPEN3D_RES_DIR}")
  add_custom_command(TARGET run_skeleton POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:run_skeleton>/resources"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OPEN3D_RES_DIR}" "$<TARGET_FILE_DIR:run_skeleton>/resources"
    COMMENT "Copying Open3D resources from ${OPEN3D_RES_DIR}"
  )
else()
  message(WARNING
    "Open3D resources not found automatically. "
    "Set -DOPEN3D_RES_DIR=/absolute/path/to/resources to enable GUI assets copy. "
    "Skipping resource copy.")
endif()

add_custom_command(TARGET run_skeleton POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:run_skeleton>/resources"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${OPEN3D_RES_DIR}" "$<TARGET_FILE_DIR:run_skeleton>/resources"
  COMMENT "Copying Open3D resources next to the binary"
)

# (Optional) Set RPATH so your binary finds libOpen3D, Qt, etc. at runtime.
# This avoids needing LD_LIBRARY_PATH when running from build tree.
set_target_properties(run_skeleton PROPERTIES
  BUILD_RPATH "$<TARGET_FILE_DIR:run_skeleton>;$<TARGET_FILE_DIR:run_skeleton>/../lib;${_open3d_prefix}/lib"
  INSTALL_RPATH "${_open3d_prefix}/lib"
)

# -------------------- tests (optional) --------------------
enable_testing()
