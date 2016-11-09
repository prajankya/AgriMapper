cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
	std_msgs
	rosserial_arduino
  rosserial_client
)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware firmware ALL)
rosserial_add_client_target(firmware firmware-upload)


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)
