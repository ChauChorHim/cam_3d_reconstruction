set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(depth_to_points SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/depth_to_points.cpp
)
target_include_directories(depth_to_points
    PRIVATE
        ${CMAKE_SOURCE_DIR}/depth_to_points/include
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)
target_link_libraries(depth_to_points
    PUBLIC
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)
