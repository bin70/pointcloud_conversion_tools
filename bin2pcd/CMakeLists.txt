cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bin2pcd)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.8 REQUIRED COMPONENT common io visualization)
include_directories(${PCL_INCLUDE_DIRS} .)
add_definitions(${PCL_DEFINITIONS})

add_executable(bin2pcd bin2pcd.cpp)
target_link_libraries(bin2pcd ${PCL_LIBRARIES})

add_executable(semantic_pcd semantic_pcd.cpp)
target_link_libraries(semantic_pcd ${PCL_LIBRARIES})
