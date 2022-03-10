set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)

add_executable(test_depth_to_points
    test_depth_to_points.cpp
)

target_link_libraries(test_depth_to_points
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libdepth_to_points.so
        ${PROJECT_SOURCE_DIR}/lib/libtimestamp_to_pose.so
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

target_include_directories(test_depth_to_points
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(test_PointCloudSaver
    test_PointCloudSaver.cpp
)

target_link_libraries(test_PointCloudSaver
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libdepth_to_points.so
        ${PROJECT_SOURCE_DIR}/lib/libtimestamp_to_pose.so
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

target_include_directories(test_PointCloudSaver
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)
