cmake_minimum_required(VERSION 3.16)
project(MSSP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# xtensor (core)
FetchContent_Declare(
  xtensor
  GIT_REPOSITORY https://github.com/xtensor-stack/xtensor.git
  GIT_TAG 0.25.0
)
FetchContent_MakeAvailable(xtensor)

# xtensor-io (for .npy, .npz, csv, etc.)
FetchContent_Declare(
  xtensor_io
  GIT_REPOSITORY https://github.com/xtensor-stack/xtensor-io.git
  GIT_TAG 0.13.0
)
FetchContent_MakeAvailable(xtensor_io)

add_executable(MSSP src/main.cpp)
target_link_libraries(MSSP PRIVATE xtensor xtensor-io)
