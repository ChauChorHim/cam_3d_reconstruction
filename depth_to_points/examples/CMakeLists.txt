set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)

add_executable(test_timestamp_to_pose
    test_timestamp_to_pose.cpp
)

target_link_libraries(test_timestamp_to_pose
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libtimestamp_to_pose.so
        Eigen3::Eigen
)

target_include_directories(test_timestamp_to_pose
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)