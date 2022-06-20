cmake_minimum_required(VERSION 3.1)
project(sensor_filter)
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

add_executable(sensor_filtering_node ./src/ros_sensor_filter.cpp)
target_link_libraries(sensor_filtering_node ${catkin_LIBRARIES})
