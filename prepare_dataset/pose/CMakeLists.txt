set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(extract_absolute_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_absolute_poses.cpp
)
target_link_libraries(extract_absolute_poses
    PRIVATE
        poses_handler
        gps_handler
)
target_include_directories(extract_absolute_poses
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------ #

add_executable(extract_relative_poses
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_relative_poses.cpp
)
target_link_libraries(extract_relative_poses
    PRIVATE
        poses_handler
        gps_handler
)
target_include_directories(extract_relative_poses
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------ #

add_executable(extract_relative_translation
    ${CMAKE_CURRENT_SOURCE_DIR}/extract_relative_translation.cpp
)
target_link_libraries(extract_relative_translation
    PRIVATE
        poses_handler
        gps_handler
)
target_include_directories(extract_relative_translation
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${EIGEN3_INCLUDE_DIRS}
)