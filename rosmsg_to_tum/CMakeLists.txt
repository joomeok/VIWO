cmake_minimum_required(VERSION 3.1)
project(odom_to_tum)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    )


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp 
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_executable(odom_to_tum_node ./src/rosmsg_to_tum.cpp)
target_link_libraries(odom_to_tum_node ${catkin_LIBRARIES})
