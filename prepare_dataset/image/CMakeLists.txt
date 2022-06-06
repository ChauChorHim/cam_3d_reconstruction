set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(undistort_images
    ${CMAKE_CURRENT_SOURCE_DIR}/undistort_images.cpp
)
target_link_libraries(undistort_images 
    PRIVATE
        images_handler
)
target_include_directories(undistort_images
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

add_executable(crop_images
    ${CMAKE_CURRENT_SOURCE_DIR}/crop_images.cpp
)
target_link_libraries(crop_images 
    PRIVATE
        images_handler
)
target_include_directories(crop_images
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

add_executable(downsample_images
    ${CMAKE_CURRENT_SOURCE_DIR}/downsample_images.cpp
)
target_link_libraries(downsample_images
    PRIVATE
        images_handler
)
target_include_directories(downsample_images
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)