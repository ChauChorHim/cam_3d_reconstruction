link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_BUILD_TYPE "Debug")
# ------------------------------------------------------------ #

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/postprocess_data)

# ------------------------------------------------------------ #

add_executable(one_frame
    ${CMAKE_CURRENT_SOURCE_DIR}/one_frame.cpp
)

target_link_libraries(one_frame
    PRIVATE 
        files_handler
        pc_tools 
)

target_include_directories(one_frame
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${CMAKE_SOURCE_DIR}/postprocess_data/include
)

# ------------------------------------------------------------ #

add_executable(multi_frame
    ${CMAKE_CURRENT_SOURCE_DIR}/multi_frame.cpp
)

target_link_libraries(multi_frame
    PRIVATE
        files_handler
        pc_tools 
)

target_include_directories(multi_frame
    PRIVATE
        ${CMAKE_SOURCE_DIR}/prepare_dataset/include
        ${CMAKE_SOURCE_DIR}/postprocess_data/include
)

# ------------------------------------------------------------ #