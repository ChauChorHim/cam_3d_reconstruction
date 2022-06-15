SET(CMAKE_BUILD_TYPE "Debug")

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


add_executable(icp
    ${CMAKE_CURRENT_SOURCE_DIR}/icp.cpp
)
target_link_libraries(icp
    PRIVATE
    pc_tools
    poses_handler
)
target_include_directories(icp
    PRIVATE
    ${CMAKE_SOURCE_DIR}/prepare_dataset/include
    ${CMAKE_SOURCE_DIR}/postprocess_data/include
)


# add_executable(test_ndt
#     ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
# )
# target_link_libraries(test_ndt
#     PRIVATE
#     ${PCL_LIBRARIES}
# )
# target_include_directories(test_ndt
#     PRIVATE
#     ${PCL_INCLUDE_DIRS}
# )