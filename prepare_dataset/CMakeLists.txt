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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin)

# ------------------------------------------------------------ #

add_executable(undistort_images
    ${CMAKE_SOURCE_DIR}/examples/src/undistort_images.cpp
)
target_link_libraries(undistort_images 
    ${OpenCV_LIBS} 
)

# ------------------------------------------------------------ #

add_executable(crop_images
    ${CMAKE_SOURCE_DIR}/examples/src/crop_images.cpp
)
target_link_libraries(crop_images 
    ${OpenCV_LIBS} 
)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_SOURCE_DIR}/examples/src/make_train_val_files.cpp
)

# ------------------------------------------------------------ #

add_executable(extract_poses
    ${CMAKE_SOURCE_DIR}/examples/src/extract_poses.cpp
)

# ------------------------------------------------------------ #

# add_executable(relative_to_absolute_poses
#     ${CMAKE_SOURCE_DIR}/examples/src/relative_to_absolute_poses.cpp
# )

# ------------------------------------------------------------ #

# add_executable(remove_static_scene
#     ${CMAKE_SOURCE_DIR}/examples/src/remove_static_scene.cpp
# )
# target_link_libraries(remove_static_scene 
#     ${OpenCV_LIBS} 
# )

# ------------------------------------------------------------ #

add_executable(mask_moving_objects
    ${CMAKE_SOURCE_DIR}/examples/src/mask_moving_objects.cpp
)
target_link_libraries(mask_moving_objects
    ${OpenCV_LIBS}
)

# ------------------------------------------------------------ #

# add_executable(make_ba_dataset
#     ${CMAKE_SOURCE_DIR}/examples/src/make_ba_dataset.cpp
# )
# target_link_libraries(make_ba_dataset
#     ${OpenCV_LIBS}
# )

# add_executable(check_gps_file
#     ${CMAKE_SOURCE_DIR}/examples/src/check_gps_file.cpp
# )

# ------------------------------------------------------------ #

add_executable(process_npy_mask
    ${CMAKE_SOURCE_DIR}/examples/src/process_npy_mask.cpp
)
target_link_libraries(process_npy_mask
    ${OpenCV_LIBS}
)