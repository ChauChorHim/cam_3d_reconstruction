set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(extract_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_poses.cpp
)
target_link_libraries(extract_poses
    poses_handler
    gps_handler
)