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
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/../../prepare_dataset/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin)
set (CMAKE_BUILD_TYPE "Debug")

# ------------------------------------------------------------ #

add_executable(one_frame
    examples/src/one_frame.cpp
)

target_link_libraries(one_frame
    PRIVATE 
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# ------------------------------------------------------------ #

add_executable(multi_frame
    examples/src/multi_frame.cpp
)

target_link_libraries(multi_frame
    PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# ------------------------------------------------------------ #