cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

#Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARIES_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})


#Build our plugin
add_library(hummingbot_plugin SHARED hummingbot_plugin.cc)
target_link_libraries(hummingbot_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
