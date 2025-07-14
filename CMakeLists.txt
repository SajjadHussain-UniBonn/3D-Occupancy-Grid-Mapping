cmake_minimum_required(VERSION 3.22.1)
project(3d_occupancy_grid_mapping LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED HINTS ${CMAKE_SOURCE_DIR}/open3d/lib/cmake/Open3D)

add_subdirectory(dataloader)
add_subdirectory(visualizer)
add_subdirectory(app)

