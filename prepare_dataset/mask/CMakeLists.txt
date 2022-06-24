set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/prepare_dataset)

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
