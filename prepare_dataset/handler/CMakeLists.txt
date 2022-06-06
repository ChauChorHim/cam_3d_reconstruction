set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(files_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/files_handler.cpp
)
target_include_directories(files_handler
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

add_library(gps_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/gps_handler.cpp
)
target_include_directories(gps_handler
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)

# ------------------------------------------------------------ #

add_library(images_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/images_handler.cpp
)
target_include_directories(images_handler
    PRIVATE
        ${OPENCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)
target_link_libraries(images_handler 
    PUBLIC
        ${OpenCV_LIBS}
        files_handler
)

# ------------------------------------------------------------ #

add_library(poses_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/poses_handler.cpp
)
target_include_directories(poses_handler
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------ #

add_library(opticalFlow_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/opticalFlow_handler.cpp
)
target_include_directories(opticalFlow_handler
    PRIVATE
        ${OPENCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
)
target_link_libraries(opticalFlow_handler PUBLIC
    ${OpenCV_LIBS}
)