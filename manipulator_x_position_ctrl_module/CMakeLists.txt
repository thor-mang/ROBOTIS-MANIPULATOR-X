################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_position_ctrl_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  robotis_math
  roscpp
  std_msgs
  sensor_msgs
  cmake_modules

  manipulator_x_position_ctrl_module_msgs
)

find_package(Eigen REQUIRED)
find_package(Orocos-KDL)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manipulator_x_position_ctrl_module
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Orocos-KDL_INCLUDE_DIRS}
)

add_library(manipulator_x_position_ctrl_module src/position_ctrl_module.cpp)
add_dependencies(manipulator_x_position_ctrl_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_x_position_ctrl_module yaml-cpp /opt/ros/kinetic/lib/liborocos-kdl.so ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
