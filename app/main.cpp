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
    const PoseAndCloud pose_and_cloud = dataset[0];
    visualize(pose_and_cloud.second);
    return 0;
}

