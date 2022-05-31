link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/depth_to_points)

# ------------------------------------------------------------ #

add_executable(one_frame
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/one_frame.cpp
)

target_link_libraries(one_frame
    PRIVATE 
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        files_handler
)

# ------------------------------------------------------------ #

add_executable(multi_frame
    examples/multi_frame.cpp
)

target_link_libraries(multi_frame
    PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        files_handler
)

# ------------------------------------------------------------ #