cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(pose2tf src/pose2tf.cpp)
target_link_libraries(pose2tf ${catkin_LIBRARIES})
