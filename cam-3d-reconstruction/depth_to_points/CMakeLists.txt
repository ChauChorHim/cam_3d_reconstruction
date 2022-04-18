cmake_minimum_required(VERSION 3.5)
project(depth_to_points VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}   
    ${CMAKE_SOURCE_DIR}/include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(timestamp_to_pose SHARED
    src/timestamp_to_pose.cpp
)

# ------------------------------------------------------------ #

add_library(depth_to_points SHARED
    src/depth_to_points.cpp
)

target_link_libraries(depth_to_points
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# ------------------------------------------------------------ #

add_library(rotation_handler SHARED
    src/rotation_handler.cpp
)

# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin)
set (CMAKE_BUILD_TYPE "Debug")

# ------------------------------------------------------------ #

add_executable(test_one_frame
    examples/src/test_one_frame.cpp
)

target_link_libraries(test_one_frame
    PRIVATE 
        depth_to_points
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# ------------------------------------------------------------ #

add_executable(test_multi_frame
    examples/src/test_multi_frame.cpp
)

target_link_libraries(test_multi_frame
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libfiles_handler.so
        ${PROJECT_SOURCE_DIR}/lib/librotation_handler.so
        timestamp_to_pose
        depth_to_points
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# ------------------------------------------------------------ #