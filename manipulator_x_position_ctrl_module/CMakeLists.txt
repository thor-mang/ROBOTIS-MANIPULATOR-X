cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_position_ctrl_module)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  manipulator_x_position_ctrl_module_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manipulator_x_position_ctrl_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  /opt/ros/kinetic/include/kdl
)

## Declare a C++ library
add_library(manipulator_x_position_ctrl_module
  src/position_ctrl_module.cpp
)

add_dependencies(manipulator_x_position_ctrl_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_x_position_ctrl_module
  yaml-cpp
  ${catkin_LIBRARIES}
  /opt/ros/kinetic/lib/liborocos-kdl.so
)
