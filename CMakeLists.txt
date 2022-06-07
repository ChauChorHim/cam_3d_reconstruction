cmake_minimum_required(VERSION 3.5)
project(depth_to_points VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
message(FATAL_ERROR “In-source build detected!”)
endif()

add_subdirectory(prepare_dataset)
add_subdirectory(depth_to_points)
add_subdirectory(postprocess_data)