cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper)

find_package(catkin REQUIRED COMPONENTS
  agri_mapper_base
  agri_mapper_bringup
  agri_mapper_description
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
