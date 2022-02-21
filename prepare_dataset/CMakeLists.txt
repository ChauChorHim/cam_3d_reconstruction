cmake_minimum_required(VERSION 3.5)
project(prepare_dataset VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(images_handler SHARED 
    src/images_handler.cpp
)
target_include_directories(images_handler
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(images_handler 
    ${OpenCV_LIBS}
)


add_library(files_handler SHARED
    src/files_handler.cpp
)
target_include_directories(files_handler
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)


add_library(images_handler_static STATIC
    src/images_handler.cpp
)
target_include_directories(images_handler_static
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(images_handler_static 
    ${OpenCV_LIBS}
)
set_target_properties(images_handler_static PROPERTIES OUTPUT_NAME "images_handler")


add_library(files_handler_static STATIC
    src/files_handler.cpp
)
target_include_directories(files_handler_static
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)
set_target_properties(files_handler_static PROPERTIES OUTPUT_NAME "files_handler")


include_directories(
    ${PROJECT_SOURCE_DIR}
)
add_executable(test
    test.cpp
)