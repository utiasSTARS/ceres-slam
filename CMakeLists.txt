cmake_minimum_required(VERSION 2.8)
project(ceres_slam)

# Set the build type. Options are:
#  Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug : w/ debug symbols, w/o optimization
#  Release : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
message("${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
# add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
# target_link_libraries(${PROJECT_NAME} ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(liegroup_test "examples/liegroup_test.cpp")
target_link_libraries(liegroup_test ${EIGEN_LIBRARIES})

add_executable(camera_test "examples/camera_test.cpp")
target_link_libraries(camera_test ${EIGEN_LIBRARIES})

add_executable(blowup_test "examples/blowup_test.cpp")
target_link_libraries(blowup_test ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})

add_executable(loop_test "examples/loop_test.cpp")
target_link_libraries(loop_test ${EIGEN_LIBRARIES} ${CERES_LIBRARIES})
