set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_CURRENT_SOURCE_DIR}/make_train_val_files.cpp
)
target_link_libraries(make_train_val_files
    PRIVATE
        files_handler
)
target_include_directories(make_train_val_files
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

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