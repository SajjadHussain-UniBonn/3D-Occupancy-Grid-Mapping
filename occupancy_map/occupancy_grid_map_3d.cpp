#include "occupancy_grid_map_3d.hpp"

OccupancyGridMap3D::OccupancyGridMap3D(const double voxel_size) : voxel_size(voxel_size) {}
Eigen::Vector3i OccupancyGridMap3D::pointToVoxel(const Eigen::Vector3d& point) const{
    return (point/voxel_size).array().floor().cast<int>();}