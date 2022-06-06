set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(extract_absolute_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_absolute_poses.cpp
)
target_link_libraries(extract_absolute_poses
    poses_handler
    gps_handler
)

# ------------------------------------------------------------ #

add_executable(extract_relative_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_relative_poses.cpp
)
target_link_libraries(extract_relative_poses
    poses_handler
    gps_handler
)

# ------------------------------------------------------------ #

add_executable(extract_relative_translation
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_relative_translation.cpp
)
target_link_libraries(extract_relative_translation
    poses_handler
    gps_handler
)