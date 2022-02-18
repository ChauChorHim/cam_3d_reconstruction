set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(undistortImages
    main.cpp)

target_link_libraries(undistortImages PRIVATE
    ${OpenCV_LIBS} 
    undistort_images)
