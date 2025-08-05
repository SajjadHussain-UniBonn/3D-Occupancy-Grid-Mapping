add_executable(app)
target_sources(app PRIVATE main.cpp)
target_compile_features(app PRIVATE cxx_std_17)
target_link_libraries(app PRIVATE dataloader_lib visualizer_lib 
                                    occupancy_grid_map_3d_lib)