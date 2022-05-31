set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")

# ------------------------------------------------------------ #

add_executable(undistort_images
    ${CMAKE_CURRENT_SOURCE_DIR}/src/undistort_images.cpp
)
target_link_libraries(undistort_images 
    images_handler
)

# ------------------------------------------------------------ #

add_executable(crop_images
    ${CMAKE_CURRENT_SOURCE_DIR}/src/crop_images.cpp
)
target_link_libraries(crop_images 
    images_handler
)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/make_train_val_files.cpp
)
target_link_libraries(make_train_val_files
    files_handler
)

# ------------------------------------------------------------ #

add_executable(extract_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/src/extract_poses.cpp
)
target_link_libraries(extract_poses
    poses_handler
    gps_handler
)