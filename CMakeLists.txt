cmake_minimum_required(VERSION 3.5)
project(depth_to_points VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}   
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
    ${CMAKE_SOURCE_DIR}/depth_to_points/include
)

add_subdirectory(prepare_dataset)
add_subdirectory(depth_to_points)
add_subdirectory(postprocess_data)