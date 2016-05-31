cmake_minimum_required(VERSION 2.8)
project(ceres-slam)

# Set the build type. Options are:
#  Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug : w/ debug symbols, w/o optimization
#  Release : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/ceres_slam/*.cpp")

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(geometry_test tests/geometry_test.cpp)
target_link_libraries(geometry_test ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(light_test tests/light_test.cpp)
target_link_libraries(light_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(dense_stereo_test tests/dense_stereo_test.cpp)
target_link_libraries(dense_stereo_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dataset_vo tests/dataset_vo.cpp)
target_link_libraries(dataset_vo ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(dataset_vo_sun tests/dataset_vo_sun.cpp)
target_link_libraries(dataset_vo_sun ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(dataset_ba_phong tests/dataset_ba_phong.cpp)
target_link_libraries(dataset_ba_phong ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

# add_custom_target(docs ALL
# 	COMMAND "doxygen" "doxygen.config"
# 	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
