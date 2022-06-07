set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/postprocess_data)

# ------------------------------------------------------------ #

add_executable(remove_static_scene
    ${CMAKE_CURRENT_SOURCE_DIR}/remove_static_scene.cpp
)
target_link_libraries(remove_static_scene 
    PRIVATE
    opticalFlow_handler
    images_handler
    files_handler
)
target_include_directories(remove_static_scene
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

add_executable(remove_small_mask
    ${CMAKE_CURRENT_SOURCE_DIR}/remove_small_mask.cpp
)
target_link_libraries(remove_small_mask
    PRIVATE
    files_handler
    ${OpenCV_LIBS}
)
target_include_directories(remove_small_mask
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

# add_executable(delete_ghost_mask
#     ${CMAKE_CURRENT_SOURCE_DIR}/delete_ghost_mask/delete_ghost_mask.cpp
# )
# target_link_libraries(delete_ghost_mask
#     files_handler
#     ${OpenCV_LIBS}
# )

# ------------------------------------------------------------ #

add_executable(depth_npy_to_jpeg
    ${CMAKE_CURRENT_SOURCE_DIR}/depth_npy_to_jpeg.cpp
)
target_link_libraries(depth_npy_to_jpeg
    PRIVATE
    files_handler
    ${OpenCV_LIBS}
)
target_include_directories(depth_npy_to_jpeg
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)