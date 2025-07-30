#include "dataloader.hpp"
#include "visualizer.hpp"
#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <utility>


using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d,Vector3dVector>;

int main()
{
    const std::string data_dir = "/home/sajjad/3D-Occupancy-Grid-Mapping/data";
    const dataloader::Dataset dataset = dataloader::Dataset(data_dir);
    const PoseAndCloud pose_and_cloud = dataset[10];
    // visualize(pose_and_cloud.second);
    // std::cout<< pose_and_cloud.first<<std::endl;
    Eigen::Matrix4d pose = pose_and_cloud.first;
    Eigen::Vector3d sensor_origin = pose.block<3, 1>(0, 3);
    std::vector<Eigen::Vector3d> points = pose_and_cloud.second;
    Eigen::Vector3d p = points[0];
    Eigen::Vector4d ph(p.x(), p.y(), p.z(), 1.0);
    Eigen::Vector4d map_point = pose * ph;
    std::cout<<map_point<<std::endl;
    return 0;
}

