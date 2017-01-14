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
	grid_map_ros
  grid_map_cv
  grid_map_msgs
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
	${EIGEN3_INCLUDE_DIR}
)

add_executable(scan_clipper src/scan_clipper.cpp)
target_link_libraries(scan_clipper ${catkin_LIBRARIES})

add_executable(odom_calc src/odom_calc.cpp)
target_link_libraries(odom_calc ${catkin_LIBRARIES})

add_executable(imu_calc src/imu_calc.cpp)
target_link_libraries(imu_calc ${catkin_LIBRARIES})

add_executable(grid_map_demo src/grid_map_demo.cpp)
target_link_libraries(grid_map_demo ${catkin_LIBRARIES})
