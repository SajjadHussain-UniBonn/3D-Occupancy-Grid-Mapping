#include "dataloader.hpp"
#include "visualizer.hpp"
#include "occupancy_grid_map_3d.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>

int main(){

    const std::string data_dir = "/home/sajjad/3D-Occupancy-Grid-Mapping/data";
    const dataloader::Dataset dataset = dataloader::Dataset(data_dir);
    OccupancyGridMap3D map(1.0);
    double total_scan_time = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i=0; i< dataset.size(); ++i){
        const auto& [pose,cloud] = dataset[i];
        auto start =  std::chrono::high_resolution_clock::now();
        map.IntegrateScan(pose,cloud);
        auto end = std::chrono::high_resolution_clock::now();
        total_scan_time += std::chrono::duration<double>(end - start).count();

        double progress = (static_cast<double>(i+1) / dataset.size()) * 100.0;
        std::cout << "\rProgress: " << std::fixed << std::setprecision(1) << progress << "% completed" << std::flush;
    }    
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "\nAverage scan time: " << total_scan_time / dataset.size() << " s\n";
    std::cout << "Total execution time: " << std::chrono::duration<double>(t1 - t0).count() << " s\n";
    visualize(map.ExtractOccupiedVoxels());

    return 0;
}

