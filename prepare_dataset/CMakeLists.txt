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


SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")


# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ------------------------------------------------------------ #

add_executable(process_images
    ${CMAKE_SOURCE_DIR}/examples/process_images.cpp
)
target_link_libraries(process_images 
    ${OpenCV_LIBS} 
)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_SOURCE_DIR}/examples/make_train_val_files.cpp
)

# ------------------------------------------------------------ #

add_executable(extract_poses
    ${CMAKE_SOURCE_DIR}/examples/extract_poses.cpp
)

# ------------------------------------------------------------ #

add_executable(remove_static_scene
    ${CMAKE_SOURCE_DIR}/examples/remove_static_scene.cpp
)
target_link_libraries(remove_static_scene 
    ${OpenCV_LIBS} 
)

# ------------------------------------------------------------ #

add_executable(make_ba_dataset
    ${CMAKE_SOURCE_DIR}/examples/make_ba_dataset.cpp
)
target_link_libraries(make_ba_dataset
    ${OpenCV_LIBS}
)