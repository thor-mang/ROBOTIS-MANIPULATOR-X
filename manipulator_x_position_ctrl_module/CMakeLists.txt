################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_position_ctrl_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  robotis_device
  robotis_framework_common
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(manipulator_x_position_ctrl_module src/position_ctrl_module.cpp)
add_dependencies(manipulator_x_position_ctrl_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_x_position_ctrl_module yaml-cpp ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
