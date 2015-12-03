cmake_minimum_required(VERSION 2.8.3)
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
	std_msgs
	sensor_msgs
	geometry_msgs
	image_transport
	cv_bridge
	message_filters
	rosconsole
	cmake_modules
	tf
	tf_conversions
)

find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		roscpp
		std_msgs
		sensor_msgs
		geometry_msgs
		image_transport
		cv_bridge
		message_filters
		rosconsole
		cmake_modules
		tf
		tf_conversions
	DEPENDS
		Eigen
		Ceres
)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# add_executable(ceres_odometry_node src/nodes/odometry_node.cpp)
# target_link_libraries(ceres_odometry_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(camera_test src/nodes/camera_test.cpp)
target_link_libraries(camera_test ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(geometry_check src/ceres_slam/geometry_check.cpp)
target_link_libraries(geometry_check ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
