# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/postprocess_data)

# ------------------------------------------------------------ #

add_executable(remove_static_scene
    ${CMAKE_CURRENT_SOURCE_DIR}/remove_static_scene/remove_static_scene.cpp
)
target_link_libraries(remove_static_scene 
    images_handler
    opticalFlow_handler
    files_handler
)

# ------------------------------------------------------------ #

# add_executable(mask_moving_objects
#     ${CMAKE_SOURCE_DIR}/examples/src/mask_moving_objects.cpp
# )
# target_link_libraries(mask_moving_objects
#     ${OpenCV_LIBS}
# )

# ------------------------------------------------------------ #

add_executable(remove_small_mask
    ${CMAKE_CURRENT_SOURCE_DIR}/remove_small_mask/remove_small_mask.cpp
)
target_link_libraries(remove_small_mask
    files_handler
    ${OpenCV_LIBS}
)

# ------------------------------------------------------------ #

# add_executable(delete_ghost_mask_jpeg
#     ${CMAKE_SOURCE_DIR}/examples/src/delete_ghost_mask_jpeg.cpp
# )

# ------------------------------------------------------------ #

add_executable(depth_npy_to_jpeg
    ${CMAKE_CURRENT_SOURCE_DIR}/depth_npy_to_jpeg/depth_npy_to_jpeg.cpp
)
target_link_libraries(depth_npy_to_jpeg
    files_handler
    ${OpenCV_LIBS}
)