cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance_turtlebot)

# Compile as C++11, supported in ROS Kinetic
add_compile_options(-std=c++11)

# Find catkin and any catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  geometry_msgs
)

# Declare a catkin package
catkin_package()

# Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS})

# Build algo and main
add_executable(obstacle_avoidance_turtlebot src/algo.cpp src/main.cpp)
target_link_libraries(obstacle_avoidance_turtlebot ${catkin_LIBRARIES})
