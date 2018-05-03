cmake_minimum_required(VERSION 2.8.3)
project(pr1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(navi src/navi.cpp)
target_link_libraries(navi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

