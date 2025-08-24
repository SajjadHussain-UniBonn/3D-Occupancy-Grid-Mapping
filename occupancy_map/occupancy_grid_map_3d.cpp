#include "occupancy_grid_map_3d.hpp"
#include<algorithm>



OccupancyGridMap3D::OccupancyGridMap3D(const double voxel_size) : voxel_size(voxel_size) {}
Eigen::Vector3i OccupancyGridMap3D::PointToVoxel(const Eigen::Vector3d& point) const{
    return (point/voxel_size).array().floor().cast<int>();
}
std::vector<Eigen::Vector3i> OccupancyGridMap3D::Bresenham3D(const Eigen::Vector3i& start, const Eigen::Vector3i& end) const{
    std::vector<Eigen::Vector3i> voxels;
    Eigen::Vector3i p = start;
    Eigen::Vector3i d = (end-start).cwiseAbs();
    Eigen::Vector3i s = (end-start).cwiseSign();
    int dx = d.x(), dy = d.y(), dz = d.z();
    int x = p.x(), y = p.y(), z = p.z();
    int dx2 = 2*dx , dy2 = 2*dy, dz2 = 2*dz;
    if (dx >= dy && dx >= dz){
        int err_y = dy2 - dx;
        int err_z = dz2 - dx;
        for(int i =0; i<= dx; ++i){
            voxels.emplace_back(x,y,z);
            if (err_y > 0) {y += s.y(); err_y -= dx2;}
            if (err_z > 0) {z += s.z(); err_z -= dx2;}
            err_y += dy2;
            err_z += dz2;
            x += s.x();
        }
    }
    else if (dy >= dx && dy >= dz){ 
        int err_x = dx2 - dy;
        int err_z = dz2 - dy;
        for (int i = 0; i<=dy; ++i){
            voxels.emplace_back(x,y,z);
            if (err_x > 0) {x += s.x(); err_x -= dy2;}
            if (err_z > 0) {z += s.z(); err_z -= dy2;}
            err_x += dx2;
            err_z += dz2;
            y += s.y();
        }
        
    }
    else{
        int err_x = dx2 - dz;
        int err_y = dy2 - dz;
        for (int i = 0; i<=dz; ++i){
            voxels.emplace_back(x,y,z);
            if (err_x > 0) {x += s.x(); err_x -= dz2;}
            if (err_y > 0) {y += s.y(); err_y -= dz2;}
            err_x += dx2;
            err_y += dy2;
            z += s.z();
        }  
    }
    voxels.shrink_to_fit();
    return voxels;
}
void OccupancyGridMap3D::UpdateVoxelState (const Eigen::Vector3i& voxel,const double voxel_curr_log_odd,const double inv_sensor_model_log_odd){
    grid_map[voxel] = voxel_curr_log_odd + inv_sensor_model_log_odd - l0;
}
void OccupancyGridMap3D::IntegrateScan(const Eigen::Matrix4d& T, const std::vector<Eigen::Vector3d>& points){
    Eigen::Vector3d robot_pos = T.block<3, 1>(0, 3);
    auto start_voxel = PointToVoxel(robot_pos);
    std::for_each(points.cbegin(),points.cend(),[&](const Eigen::Vector3d& p){
        Eigen::Vector4d ph(p.x(), p.y(), p.z(), 1.0);
        Eigen::Vector3d map_point = (T * ph).head<3>();
        auto end_voxel   = PointToVoxel(map_point);
        if (grid_map.find(start_voxel) != grid_map.end() && grid_map.find(end_voxel) != grid_map.end()){
            return;
        }
        auto ray = Bresenham3D(start_voxel, end_voxel);
        if (!ray.empty()){
            std::for_each(ray.cbegin(), std::prev(ray.cend()),[&](const auto& voxel){
                auto voxel_log_odd = GetVoxelLogOdds(voxel);
                UpdateVoxelState(voxel, voxel_log_odd, l_free);
            });
            auto last_voxel = ray.back();
            UpdateVoxelState(last_voxel, GetVoxelLogOdds(last_voxel), l_occ);
        }
    });
}
Eigen::Vector3d OccupancyGridMap3D::VoxelToPoint (const Eigen::Vector3i& voxel) const{
    return voxel.cast<double>() * voxel_size + Eigen::Vector3d::Constant(voxel_size/2.0);
}
std::vector<Eigen::Vector3d> OccupancyGridMap3D::ExtractOccupiedVoxels() const {
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::for_each(grid_map.cbegin(),grid_map.cend(),[&](const auto& pair){
        if (LogOddsToProb(pair.second) >= 0.7){occupied_voxels.emplace_back(VoxelToPoint(pair.first));}});
    occupied_voxels.shrink_to_fit();
    return occupied_voxels;
}
std::vector<Eigen::Vector3d> OccupancyGridMap3D::ExtractFreeVoxels() const {
    std::vector<Eigen::Vector3d> free_voxels;
    std::for_each(grid_map.cbegin(),grid_map.cend(),[&](const auto& pair){
        if (LogOddsToProb(pair.second) <= 0.3){free_voxels.emplace_back(VoxelToPoint(pair.first));}});
    free_voxels.shrink_to_fit();
    return free_voxels;
}
double OccupancyGridMap3D::GetVoxelLogOdds(const Eigen::Vector3i& voxel) const{
    auto it = grid_map.find(voxel);
    if(it == grid_map.end()){return l0;}
    return it->second;
}
double OccupancyGridMap3D::ProbToLogOdds (const double p) const{
    // To avoid division by zero
    if (p <= 0.0) { return -std::numeric_limits<double>::infinity();}
    if (p >= 1.0) { return std::numeric_limits<double>::infinity();}
    return std::log(p/(1.0 - p));
}
double OccupancyGridMap3D::LogOddsToProb (const double l) const{
    return 1.0/(1.0 + std::exp(-l));
}