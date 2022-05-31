SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")


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

add_executable(remove_small_mask_npy
    ${CMAKE_CURRENT_SOURCE_DIR}/remove_small_mask_npy/remove_small_mask_npy.cpp
)
target_link_libraries(remove_small_mask_npy
    files_handler
    ${OpenCV_LIBS}
)

# ------------------------------------------------------------ #


# add_executable(delete_ghost_mask_jpeg
#     ${CMAKE_SOURCE_DIR}/examples/src/delete_ghost_mask_jpeg.cpp
# )