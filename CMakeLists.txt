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

file(GLOB ${PROJECT_NAME}_HEADERS "include/ceres_slam/*.h")
file(GLOB ${PROJECT_NAME}_SOURCES "src/ceres_slam/*.cpp")

add_library(${PROJECT_NAME}
	${${PROJECT_NAME}_HEADERS}
	${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# add_executable(ceres_odometry_node src/nodes/odometry_node.cpp)
# target_link_libraries(ceres_odometry_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(camera_test src/tests/camera_test.cpp)
target_link_libraries(camera_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(geometry_test src/tests/geometry_test.cpp)
target_link_libraries(geometry_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(light_test src/tests/light_test.cpp)
target_link_libraries(light_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(dataset_ba src/tests/dataset_ba.cpp)
target_link_libraries(dataset_ba ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_custom_target(docs ALL
	COMMAND "doxygen" "doxygen.config"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
