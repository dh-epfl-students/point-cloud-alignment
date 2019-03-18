project(PointCloudAlignment)
cmake_minimum_required(VERSION 3.6.2)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

find_package(PCL 1.9 REQUIRED)

# For CMake < 3.9, we need to make the target ourselves
if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${PCL_INCLUDE_DIRS}
                    ${HEADER_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(./src SRC_LIST)

message(${SRC_LIST})

add_executable(${PROJECT_NAME} main.cpp ${SRC_LIST}
    ${HEADER_DIR}/common.h
    ${HEADER_DIR}/point_normal_k.h
    ${HEADER_DIR}/plane_segmentation.h
    ${HEADER_DIR}/normal_computation.h
    ${HEADER_DIR}/plane.h
    ${HEADER_DIR}/pfh_evaluation.h
    ${HEADER_DIR}/segmented_points_container.h)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

