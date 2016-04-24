cmake_minimum_required(VERSION 2.8.3)
project(smart_ar)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	geometry_msgs
	image_transport
	roscpp
	rospy
	sensor_msgs
)

add_definitions(-std=c++11)

set(aruco_DIR /usr/local/lib/cmake)
set(CMAKE_MODULE_PATH ${aruco_DIR})
find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS src
	CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs
)

include_directories(
	src
	${catkin_INCLUDE_DIRS}
)

add_executable(ros_aruco src/ArucoDetector.cpp)

target_link_libraries(ros_aruco
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
)

file(GLOB all_ros_files src/* script/* launch/*)
add_custom_target(dummy_target2 ALL SOURCES ${all_ros_files})
