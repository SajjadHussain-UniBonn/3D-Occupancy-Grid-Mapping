add_library(dataloader_lib STATIC)
target_sources(dataloader_lib PRIVATE dataloader.cpp)
target_compile_features(dataloader_lib PRIVATE cxx_std_17)
target_include_directories(dataloader_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(dataloader_lib 
                        PUBLIC Eigen3::Eigen 
                        PRIVATE Open3D::Open3D)