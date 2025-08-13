#pragma once
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>


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
        const double l0 = 0.0; // prob = 0.5
        const double l_occ = 0.8473; //prob = 0.7
        const double l_free = - 0.8473; //prob = 0.3

        std::unordered_map<Eigen::Vector3i,double ,VoxelHash> grid_map;
        std::vector<Eigen::Vector3i> bresenham3D(const Eigen::Vector3i& start, const Eigen::Vector3i& end) const;
        Eigen::Vector3i pointToVoxel(const Eigen::Vector3d& point) const; 
        void updateVoxelState (const Eigen::Vector3i& voxel, const double voxel_curr_log_odd,const double inv_sensor_model_log_odd);
        Eigen::Vector3d voxelToPoint (const Eigen::Vector3i& voxel) const;
        double getVoxelLogOdds(const Eigen::Vector3i& voxel) const;
        double probToLogOdds (const double p) const;
        double logOddsToProb (const double l) const;

};
