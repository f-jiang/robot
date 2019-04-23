cmake_minimum_required(VERSION 2.8.3)
project(base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rosserial_arduino
  rosserial_client
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base
#  CATKIN_DEPENDS controller_manager hardware_interface roscpp rosserial_arduino rosserial_client sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)


rosserial_configure_client(
    DIRECTORY firmware
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware hello ALL)
rosserial_add_client_target(firmware hello-upload)

