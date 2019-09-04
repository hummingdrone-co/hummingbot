cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_XX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

#Build our plugin
add_library(camera_sensor_plugin SHARED camera_sensor_plugin.cc)
target_link_libraries(camera_sensor_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
