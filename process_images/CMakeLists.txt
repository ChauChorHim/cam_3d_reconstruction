cmake_minimum_required(VERSION 3.5)
project(process_image_tools VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(process_images SHARED 
    src/process_image_tools.cpp
)

target_include_directories(process_images
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(process_images ${OpenCV_LIBS})
