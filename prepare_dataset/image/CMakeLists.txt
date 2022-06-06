set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(undistort_images
    ${CMAKE_CURRENT_SOURCE_DIR}/undistort_images.cpp
)
target_link_libraries(undistort_images 
    images_handler
)

# ------------------------------------------------------------ #

add_executable(crop_images
    ${CMAKE_CURRENT_SOURCE_DIR}/crop_images.cpp
)
target_link_libraries(crop_images 
    images_handler
)

