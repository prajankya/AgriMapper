cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_ros
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
