cmake_minimum_required(VERSION 2.8.3)
project(aruco_ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs sensor_msgs cv_bridge)

find_package(OpenCV REQUIRED)

find_package(aruco REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(aruco_ekf src/aruco_ekf_node.cpp)

target_link_libraries(aruco_ekf ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})
