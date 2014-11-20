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
)

#add_message_files(FILES
#	distPose.msg
#)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime s8_common_node s8_msgs
)
include_directories(${catkin_INCLUDE_DIRS})