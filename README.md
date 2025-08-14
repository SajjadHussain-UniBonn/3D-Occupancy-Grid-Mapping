# 3D-Occupancy-Grid-Mapping
- [3D-Occupancy-Grid-Mapping](#3D-Occupancy-Grid-Mapping)
  - [Project description](#project-description)
  - [Implementation details](#implementation-details-and-result)
  - [Installer Details](#installer-details)
    - [Ubuntu](#ubuntu)
  - [Acknowledgements](#acknowledgements)


 ## Project Description

  In this project, a 3D occupancy mapping pipeline is developed for a sequence of LiDAR scans with known poses.

## Implementation Details

All the functions are generic. There is only one hyper parameter (voxel_size). It is better to set it between 0 and 1.

## Installer Details
### Ubuntu
First, you need to clone this project in any folder. You need two external libraries, Eigen3d and Open3d, to compile and execute the code successfully. Eigen3d can be installed using the following command:  
   - `sudo apt install libeigen3-dev` 

  Open3d has to be downloaded from github and extracted to the project folder. Open3d can be downloaded from [Open3d github](https://github.com/isl-org/Open3D/releases). For ubuntu system, download this one _open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz_ from the releases. Extract it to project directory and rename the folder as "open3d".  
  For Data, you have to create a folder "data" and put your laserscanner files in a subfolder "PLY". Put poses file in the "data" folder.

Finally to build and excecute the project with CMake, following steps are required:  
- make a directory(build) in the project folder(mkdir build)
- go to the build directory(cd build)
- write configuration files to build (cmake ..)
- compile the code(make)
- run the program(./app/app)

## Acknowledgements

Thanks to the following collaborators for making this project happen:

[![Saurabh](https://github.com/saurabh1002.png?size=50)](https://github.com/saurabh1002)
[![Meher](https://github.com/mehermvr.png?size=50)](https://github.com/mehermvr)

