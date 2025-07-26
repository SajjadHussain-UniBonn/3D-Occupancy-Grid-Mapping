#pragma once
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

struct voxelHash
{
    std::size_t operator()(const Eigen::Vector3i& vec)const
    {
        return ((1 << 20) - 1) & (
        vec.x()* 73856093 ^
        vec.y() * 19349663 ^
        vec.z() * 83492791);
    }
};
class OccupancyGridMap3D
{
    public:
        OccupancyGridMap3D(const double voxel_size);
    
    private:
        double voxel_size;
        Eigen::Vector3i pointToVoxel(const Eigen::Vector3d& point) const;
};