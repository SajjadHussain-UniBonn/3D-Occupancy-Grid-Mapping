#include "dataloader.hpp"
#include "visualizer.hpp"
#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include "occupancy_grid_map_3d.hpp"


using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d,Vector3dVector>;

int main()
{
    OccupancyGridMap3D map(0.2);
    const std::string data_dir = "/home/sajjad/3D-Occupancy-Grid-Mapping/data";
    const dataloader::Dataset dataset = dataloader::Dataset(data_dir);
    const PoseAndCloud pose_and_cloud = dataset[10];
    
    Eigen::Matrix4d pose = pose_and_cloud.first;
    std::vector<Eigen::Vector3d> points = pose_and_cloud.second;

    map.integrateScan(pose, points);
    std::vector<Eigen::Vector3d> occupied_points = map.extractFreePoints();
    return 0;
}

