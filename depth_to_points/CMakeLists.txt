cmake_minimum_required(VERSION 3.5)
project(depth_to_points VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


add_library(timestamp_to_pose SHARED
    src/timestamp_to_pose.cpp
)

target_link_libraries(timestamp_to_pose 
    Eigen3::Eigen
)

target_include_directories(timestamp_to_pose
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_subdirectory(examples)