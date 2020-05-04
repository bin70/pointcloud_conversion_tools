cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcap2bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(PCL_DIR /home/elvin/pcl_1_8_1/share/pcl-1.8)

find_package(PCL 1.8 REQUIRED common io visualization filters)
include_directories(${PCL_INCLUDE_DIRS} .)
add_definitions(${PCL_DEFINITIONS})

add_executable(pcap2bin pcap2bin.cpp PointCloudReader.cpp)
target_link_libraries(pcap2bin ${PCL_LIBRARIES} -lpcap)
