#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>
#include "open3d/Open3D.h"

inline void visualize(
        const std::vector<Eigen::Vector3d>& pointcloud) {
    open3d::visualization::DrawGeometries(
            {std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
}
inline void visualizeOccupancyMap(const std::vector<Eigen::Vector3d>& occupied_points,
                                     const std::vector<Eigen::Vector3d>& free_points) 
{
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& p : occupied_points) 
    {
        cloud->points_.emplace_back(p);
        cloud->colors_.emplace_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    }
    for (const auto& p : free_points) 
    {
        cloud->points_.emplace_back(p);
        cloud->colors_.emplace_back(Eigen::Vector3d(0.0, 1.0, 0.0));
    }
    open3d::visualization::DrawGeometries({cloud});
}