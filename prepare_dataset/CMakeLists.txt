cmake_minimum_required(VERSION 3.5)
project(prepare_dataset VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")

# ------------------------------------------------------------ #

add_library(images_handler SHARED 
    ${CMAKE_SOURCE_DIR}/src/images_handler.cpp
)
target_link_libraries(images_handler 
    ${OpenCV_LIBS}
)

# ------------------------------------------------------------ #

add_library(files_handler SHARED
    ${CMAKE_SOURCE_DIR}/src/files_handler.cpp
)

# ------------------------------------------------------------ #

add_library(poses_handler SHARED
    ${CMAKE_SOURCE_DIR}/src/poses_handler.cpp
)
target_link_libraries(poses_handler
    ${CMAKE_SOURCE_DIR}/lib/libtimestamp_to_pose.so
)

# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)

# ------------------------------------------------------------ #

add_executable(process_images
    ${CMAKE_SOURCE_DIR}/examples/process_images.cpp
)
target_link_libraries(process_images 
    ${OpenCV_LIBS} 
    files_handler
    images_handler
)

# ------------------------------------------------------------ #

add_executable(make_train_val_files
    ${CMAKE_SOURCE_DIR}/examples/make_train_val_files.cpp
)
target_link_libraries(make_train_val_files
    files_handler
)

# ------------------------------------------------------------ #

# add_executable(extract_poses
#     ${CMAKE_SOURCE_DIR}/examples/extract_poses.cpp
# )
# target_link_libraries(extract_poses
#     poses_handler
#     files_handler
# )

# ------------------------------------------------------------ #


add_executable(remove_static_scene
    ${CMAKE_SOURCE_DIR}/examples/remove_static_scene.cpp
)
target_link_libraries(remove_static_scene
    images_handler
    files_handler
)
