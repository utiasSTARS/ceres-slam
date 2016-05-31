cmake_minimum_required(VERSION 2.8)
project(ceres_slam)

# Set the build type. Options are:
#  Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug : w/ debug symbols, w/o optimization
#  Release : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	image_transport
	cv_bridge
	message_filters
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		sensor_msgs
		image_transport
		cv_bridge
		message_filters
)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(sparse_stereo_odometry src/nodes/sparse_stereo_odometry_node.cpp)
# target_link_libraries(sparse_stereo_odometry ${PROJECT_NAME} ${catkin_LIBRARIES})
