cmake_minimum_required(VERSION 3.0.2)
project(hummingbird_nmpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mav_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hummingbird_nmpc
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  src/hovering_example.py
  # src/nmpc_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)