cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
#  CATKIN_DEPENDS controller_manager diff_drive_controller
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
