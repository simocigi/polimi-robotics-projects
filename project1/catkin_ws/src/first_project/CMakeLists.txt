cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	message_generation
	tf
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Sector_times.msg
)

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS 
	roscpp
	std_msgs
	geometry_msgs 
	tf
	message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometer src/odometer.cpp)
# add_dependencies(odometer  ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometer ${catkin_LIBRARIES})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

add_executable(sector_times src/sector_times.cpp)
add_dependencies(sector_times first_project_generate_messages_cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})
