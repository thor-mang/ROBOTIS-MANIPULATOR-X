################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_manager)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math

  cmake_modules

  manipulator_x_position_ctrl_module
  manipulator_x_gripper_module
)

find_package(Eigen REQUIRED)

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
#  LIBRARIES manipulator_x_manager
#  CATKIN_DEPENDS roscpp
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}  
)

add_executable(manipulator_x_manager src/manipulator_x_manager.cpp)
add_dependencies(manipulator_x_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_x_manager ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
