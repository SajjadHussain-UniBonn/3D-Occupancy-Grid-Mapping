add_library(visualizer_lib INTERFACE)
target_include_directories(visualizer_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_compile_features(visualizer_lib INTERFACE cxx_std_17)
target_link_libraries(visualizer_lib INTERFACE Eigen3::Eigen Open3D::Open3D)