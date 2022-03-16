cmake_minimum_required(VERSION 3.5)
project(prepare_dataset VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(images_handler SHARED 
    src/images_handler.cpp
)
target_link_libraries(images_handler 
    ${OpenCV_LIBS}
)

# ------------------------------------------------------------ #

add_library(files_handler SHARED
    src/files_handler.cpp
)

# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)

# ------------------------------------------------------------ #

add_executable(process_images
    examples/process_images.cpp
)
target_link_libraries(process_images 
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libfiles_handler.so
        ${PROJECT_SOURCE_DIR}/lib/libimages_handler.so
        ${OpenCV_LIBS} 
)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    examples/make_train_val_files.cpp
)
target_link_libraries(make_train_val_files 
    PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/libfiles_handler.so
)