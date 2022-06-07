set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/postprocess_data)

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