cmake_minimum_required(VERSION 3.5)
project(depth_to_points VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_library(depth_to_points SHARED
    src/depth_to_points.cpp
)

target_link_libraries(depth_to_points
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

target_include_directories(depth_to_points
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_library(cnpy SHARED
    src/cnpy.cpp
)
target_include_directories(cnpy
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)


add_subdirectory(examples)