project(PointCloudAlignment)
cmake_minimum_required(VERSION 3.6.2)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

find_package(PCL 1.9 REQUIRED)

set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${PCL_INCLUDE_DIRS}
                    ${HEADER_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(./src SRC_LIST)

message(${SRC_LIST})

add_executable(${PROJECT_NAME} main.cpp ${SRC_LIST}
    ${HEADER_DIR}/common.h
    ${HEADER_DIR}/plane_segmentation.h
    ${HEADER_DIR}/normal_computation.h)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

