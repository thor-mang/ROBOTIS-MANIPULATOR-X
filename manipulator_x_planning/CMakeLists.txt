################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_planning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  robotis_controller_msgs
  cmake_modules
  manipulator_x_position_ctrl_module_msgs
  robotis_controller_msgs
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
  INCLUDE_DIRS include
  LIBRARIES manipulator_x_planning
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS EIGEN3
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
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(manipulator_x_planning src/manipulator_x_planning.cpp)
add_dependencies(manipulator_x_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_x_planning ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
