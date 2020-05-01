cmake_minimum_required(VERSION 2.8.3)
project(robot_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosserial_arduino
  pid
  dynamic_reconfigure
)

find_package(robot_constants REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/Setpoint.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base
 CATKIN_DEPENDS controller_manager
                hardware_interface
                roscpp
                rospy
                sensor_msgs
                std_msgs
                rosserial_arduino
                pid
                dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${robot_constants_INCLUDE_DIRS}
)

add_executable(base_node src/base.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})

