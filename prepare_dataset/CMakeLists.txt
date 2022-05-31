# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")


# ------------------------------------------------------------ #

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# ------------------------------------------------------------ #

add_library(files_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/handler/files_handler.cpp
)

# ------------------------------------------------------------ #

add_library(gps_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/handler/gps_handler.cpp
)

# ------------------------------------------------------------ #

add_library(images_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/handler/images_handler.cpp
)
target_link_libraries(images_handler PUBLIC
    ${OpenCV_LIBS}
    files_handler
)

# ------------------------------------------------------------ #

add_library(poses_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/handler/poses_handler.cpp
)

# ------------------------------------------------------------ #

add_library(opticalFlow_handler SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/handler/opticalFlow_handler.cpp
)
target_link_libraries(opticalFlow_handler PUBLIC
    ${OpenCV_LIBS}
)

add_subdirectory(examples)