#include "occupancy_grid_map_3d.hpp"


OccupancyGridMap3D::OccupancyGridMap3D(const double voxel_size) : voxel_size(voxel_size) {}

Eigen::Vector3i OccupancyGridMap3D::pointToVoxel(const Eigen::Vector3d& point) const
{
    return (point/voxel_size).array().floor().cast<int>();
}

std::vector<Eigen::Vector3i> OccupancyGridMap3D::bresenham3D(const Eigen::Vector3i& start, const Eigen::Vector3i& end) const
{
    std::vector<Eigen::Vector3i> voxels;
    Eigen::Vector3i p = start;
    Eigen::Vector3i d = (end-start).cwiseAbs();
    Eigen::Vector3i s = (end-start).cwiseSign();
    int dx = d.x(), dy = d.y(), dz = d.z();
    int x = p.x(), y = p.y(), z = p.z();
    int dx2 = 2*dx , dy2 = 2*dy, dz2 = 2*dz;
    if (dx >= dy && dx >= dz)
    {
        int err_y = dy2 - dx;
        int err_z = dz2 - dx;
        for(int i =0; i<= dx; ++i)
        {
            voxels.emplace_back(x,y,z);
            if (err_y > 0) {y += s.y(); err_y -= dx2;}
            if (err_z > 0) {z += s.z(); err_z -= dx2;}
            err_y += dy2;
            err_z += dz2;
            x += s.x();
        }
    }
    else if (dy >= dx && dy >= dz)
    { 
        int err_x = dx2 - dy;
        int err_z = dz2 - dy;
        for (int i = 0; i<=dy; ++i)
        {
            voxels.emplace_back(x,y,z);
            if (err_x > 0) {x += s.x(); err_x -= dy2;}
            if (err_z > 0) {z += s.z(); err_z -= dy2;}
        }
        err_x += dx2;
        err_z += dz2;
        y += s.y();
    }
    else
    {
        int err_x = dx2 - dz;
        int err_y = dy2 - dz;
        for (int i = 0; i<=dz; ++i)
        {
            voxels.emplace_back(x,y,z);
            if (err_x > 0) {x += s.x(); err_x -= dz2;}
            if (err_y > 0) {y += s.y(); err_y -= dz2;}
        }
        err_x += dx2;
        err_y += dy2;
        z += s.z();
    }
    voxels.shrink_to_fit();
    return voxels;
}

void OccupancyGridMap3D::updateVoxelState (const Eigen::Vector3i& voxel, VoxelState state)
{
    auto it = grid_map.find(voxel);
    if (it!= grid_map.end() && state == VoxelState::free && it->second == VoxelState::occupied)
    {
        return;
    }   
    grid_map[voxel]= state;
}

void OccupancyGridMap3D::integrateScan(const Eigen::Matrix4d& pose, const std::vector<Eigen::Vector3d>& points)
{
    Eigen::Vector3d robot_pos = pose.block<3, 1>(0, 3);
    for (const auto& p: points)
    {
        Eigen::Vector4d ph(p.x(), p.y(), p.z(), 1.0);
        Eigen::Vector3d map_point = (pose * ph).head<3>();
        auto start_voxel = pointToVoxel(robot_pos);
        auto end_voxel = pointToVoxel(map_point);
        auto ray = bresenham3D(start_voxel,end_voxel);
        for (size_t i =0 ; i+1 < ray.size(); ++i)
        {
            updateVoxelState(ray[i], VoxelState::free);
        }
        updateVoxelState(ray.back(),VoxelState::occupied);
    }
}
Eigen::Vector3d OccupancyGridMap3D::voxelToPoint (const Eigen::Vector3i& voxel) const
{
    return voxel.cast<double>() * voxel_size + Eigen::Vector3d::Constant(voxel_size/2.0);
}
std::vector<Eigen::Vector3d> OccupancyGridMap3D::extractOccupiedVoxels() const 
{
    std::vector<Eigen::Vector3d> occupiedVoxels;
    auto isOccupied = [](VoxelState state) 
    {
        return state == VoxelState::occupied;
    };

    for (const auto& [voxel, state] : grid_map) 
    {
        if (isOccupied(state))
        {
            occupiedVoxels.emplace_back(voxelToPoint(voxel));
        }
    }
    occupiedVoxels.shrink_to_fit();
    return occupiedVoxels;
}
std::vector<Eigen::Vector3d> OccupancyGridMap3D::extractFreeVoxels() const 
{
    std::vector<Eigen::Vector3d> freeVoxels;
    auto isFree = [](VoxelState state) 
    {
        return state == VoxelState::free;
    };

    for (const auto& [voxel, state] : grid_map) 
    {
        if (isFree(state))
        {
            freeVoxels.emplace_back(voxelToPoint(voxel));
        }
    }
    freeVoxels.shrink_to_fit();
    return freeVoxels;
}
