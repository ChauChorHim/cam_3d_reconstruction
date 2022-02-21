cmake_minimum_required(VERSION 3.5)
project(example VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    /home/Autowise/zcq_tools/process_images/include
)

add_executable(process_images
    process_images.cpp
)

add_executable(make_train_val_files
    make_train_val_files.cpp
)

target_link_libraries(process_images PRIVATE
    ${OpenCV_LIBS} 
    /home/Autowise/zcq_tools/process_images/lib/libprocess_images.so
)

target_link_libraries(make_train_val_files PRIVATE
    ${OpenCV_LIBS} 
    /home/Autowise/zcq_tools/process_images/lib/libprocess_images.so
)
