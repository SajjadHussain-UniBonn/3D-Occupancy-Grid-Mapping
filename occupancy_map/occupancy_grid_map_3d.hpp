#pragma once
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

enum class VoxelState {unknown, free, occupied};

struct VoxelHash
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
        void integrateScan(const Eigen::Matrix4d& pose, const std::vector<Eigen::Vector3d>& points);
        std::vector<Eigen::Vector3d> extractOccupiedVoxels() const;
        std::vector<Eigen::Vector3d> extractFreeVoxels() const;

    private:
        double voxel_size;
        std::unordered_map<Eigen::Vector3i,VoxelState,VoxelHash> grid_map;
        std::vector<Eigen::Vector3i> bresenham3D(const Eigen::Vector3i& start, const Eigen::Vector3i& end) const;
        Eigen::Vector3i pointToVoxel(const Eigen::Vector3d& point) const; 
        void updateVoxelState (const Eigen::Vector3i& voxel, VoxelState state);
        Eigen::Vector3d voxelToPoint (const Eigen::Vector3i& voxel) const; 
};
