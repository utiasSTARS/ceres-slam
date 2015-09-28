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
	cmake_modules
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
		cmake_modules
	DEPENDS
		Eigen
		Ceres
)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

include_directories(
  include
  ${CATKIN_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/stereo_camera.cpp
							src/stereo_reprojection_error.cpp)
target_link_libraries(${PROJECT_NAME} ${CATKIN_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME} ${CATKIN_LIBRARIES} ${EIGEN_LIBRARIES})
