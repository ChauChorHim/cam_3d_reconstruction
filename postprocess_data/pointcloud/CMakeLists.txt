SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/postprocess_data)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

add_library(pc_tools SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/pc_tools.cpp
)
target_link_libraries(pc_tools
    PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    files_handler
)
target_include_directories(pc_tools
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
    ${CMAKE_SOURCE_DIR}/postprocess_data/include
)


add_executable(optimize_pose_with_pc
    ${CMAKE_CURRENT_SOURCE_DIR}/optimize_pose_with_pc.cpp
)
target_link_libraries(optimize_pose_with_pc
    PRIVATE
    pc_tools
    poses_handler
)
target_include_directories(optimize_pose_with_pc
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
    ${CMAKE_SOURCE_DIR}/postprocess_data/include
)