cmake_minimum_required(VERSION 2.8.3)
project(exercise3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(map_goals src/map_goals.cpp)
target_link_libraries(map_goals ${catkin_LIBRARIES})

