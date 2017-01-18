cmake_minimum_required(VERSION 2.8.3)
project(agri_mapper_base)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	roscpp
  rospy
  tf
  nav_msgs
	std_msgs
  sensor_msgs
	cv_bridge
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

add_executable(scan_clipper src/scan_clipper.cpp)
target_link_libraries(scan_clipper ${catkin_LIBRARIES})

add_executable(odom_calculator src/odom_calculator.cpp)
target_link_libraries(odom_calculator ${catkin_LIBRARIES})

add_executable(imu_calculator src/imu_calculator.cpp)
target_link_libraries(imu_calculator ${catkin_LIBRARIES})
