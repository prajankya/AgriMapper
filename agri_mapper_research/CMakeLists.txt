cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper_research)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
	sensor_msgs
  roscpp
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agri_mapper_reu
#  CATKIN_DEPENDS nav_msgs opencv2 roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tree_detector src/tree_detector.cpp)
target_link_libraries(tree_detector ${catkin_LIBRARIES})
