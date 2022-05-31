cmake_minimum_required(VERSION 3.5)
project(prepare_dataset VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)


# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")


# ------------------------------------------------------------ #

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(files_handler SHARED
    handler/files_handler.cpp
)

# ------------------------------------------------------------ #

add_library(gps_handler SHARED
    handler/gps_handler.cpp
)

# ------------------------------------------------------------ #

add_library(images_handler SHARED
    handler/images_handler.cpp
)
target_link_libraries(images_handler PUBLIC
    ${OpenCV_LIBS}
    files_handler
)

# ------------------------------------------------------------ #

add_library(poses_handler SHARED
    handler/poses_handler.cpp
)

# ------------------------------------------------------------ #

add_library(opticalFlow_handler SHARED
    handler/opticalFlow_handler.cpp
)
target_link_libraries(opticalFlow_handler PUBLIC
    ${OpenCV_LIBS}
)

add_subdirectory(examples)