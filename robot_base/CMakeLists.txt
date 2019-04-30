cmake_minimum_required(VERSION 2.8.3)
project(robot_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
  std_msgs
  serial
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base
 CATKIN_DEPENDS controller_manager hardware_interface roscpp sensor_msgs std_msgs serial
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

