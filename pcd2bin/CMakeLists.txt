cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.8 REQUIRED COMPONENT common io visualization)
include_directories(${PCL_INCLUDE_DIRS} .)
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2bin pcd2bin.cpp)
target_link_libraries(pcd2bin ${PCL_LIBRARIES})

