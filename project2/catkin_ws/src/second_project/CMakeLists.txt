cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	nav_msgs
	tf
	tf2_ros
	message_filters
	#slam_gmapping
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS 
	roscpp
	std_msgs
	sensor_msgs
	nav_msgs
	tf
	tf2_ros
	message_filters
	#slam_gmapping
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})
add_dependencies(mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(convert_odom src/convert_odom.cpp)
target_link_libraries(convert_odom ${catkin_LIBRARIES})
