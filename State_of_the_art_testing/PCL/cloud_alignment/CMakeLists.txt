project(cloud_alignment)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    "${CMAKE_CURRENT_SOURCE_DIR}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST}
                feature_cloud.h
                template_alignment.h)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

