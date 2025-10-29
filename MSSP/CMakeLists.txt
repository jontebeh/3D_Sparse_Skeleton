cmake_minimum_required(VERSION 3.16)
project(MSSP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# --- xtl (required by xtensor) ---
# xtensor 0.25.0 works with xtl 0.7.5
find_package(xtl 0.7.5 QUIET)
if(NOT xtl_FOUND)
  FetchContent_Declare(
    xtl
    GIT_REPOSITORY https://github.com/xtensor-stack/xtl.git
    GIT_TAG 0.7.5
  )
  FetchContent_MakeAvailable(xtl)
  # Ensure downstream find_package can see it
  list(APPEND CMAKE_PREFIX_PATH "${xtl_BINARY_DIR}")
endif()

# --- xtensor (core) ---
FetchContent_Declare(
  xtensor
  GIT_REPOSITORY https://github.com/xtensor-stack/xtensor.git
  GIT_TAG 0.25.0
)
FetchContent_MakeAvailable(xtensor)

# --- xtensor-io (NPY/NPZ/CSV) ---
# NOTE: needs zlib and libzip at link time.
# On Ubuntu: sudo apt-get install zlib1g-dev libzip-dev
FetchContent_Declare(
  xtensor_io
  GIT_REPOSITORY https://github.com/xtensor-stack/xtensor-io.git
  GIT_TAG 0.13.0
)
FetchContent_MakeAvailable(xtensor_io)

add_executable(MSSP src/main.cpp)
target_link_libraries(MSSP PRIVATE xtensor xtensor-io)
