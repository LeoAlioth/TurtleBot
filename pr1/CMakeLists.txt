
cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(pr1)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf
	dynamic_reconfigure
	genmsg
	nodelet
	nodelet_topic_tools
	pcl_conversions
  	pcl_msgs
	pluginlib
  	rosbag
  	rosconsole
	roslib
	tf2_ros
	tf2_geometry_msgs
)

find_package( OpenCV REQUIRED )

catkin_package()


## Add include directories
include_directories(
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  )
link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(navi src/navi.cpp)
add_executable(cylinder_detect src/cylinder_detect.cpp)
target_link_libraries(navi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(cylinder_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})