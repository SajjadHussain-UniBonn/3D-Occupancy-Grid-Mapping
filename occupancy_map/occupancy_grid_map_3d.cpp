#include "occupancy_grid_map_3d.hpp"


OccupancyGridMap3D::OccupancyGridMap3D(const double voxel_size) : voxel_size(voxel_size) {}
Eigen::Vector3i OccupancyGridMap3D::pointToVoxel(const Eigen::Vector3d& point) const{
    return (point/voxel_size).array().floor().cast<int>();}

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
            if (err_y > 0) { y += s.y();err_y -= dx2;}
            if (err_z > 0) {z += s.z(); err_z -= dx2;}
            err_y += dy2;
            err_z += dz2;
            x += s.x();
        }
    }
    else if (dy >= dx && dy >> dz)
    {

    }
}