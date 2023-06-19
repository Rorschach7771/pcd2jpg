cmake_minimum_required(VERSION 3.0)
project(pcl_process)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})



add_executable(pcd2jpg pcd2jpg.cpp)

target_link_libraries(pcd2jpg ${PCL_LIBRARIES} ${OpenCV_LIBS} ) 
