set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_master)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    roscpp
    std_msgs
    message_generation
    s8_common_node
    s8_msgs
    s8_object_aligner
    actionlib
    s8_explorer
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime s8_common_node s8_msgs s8_object_aligner actionlib s8_explorer
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(s8_master_node src/master_node.cpp)
target_link_libraries(s8_master_node ${catkin_LIBRARIES})