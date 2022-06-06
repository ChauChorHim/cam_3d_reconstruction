set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_CURRENT_SOURCE_DIR}/make_train_val_files.cpp
)
target_link_libraries(make_train_val_files
    files_handler
)