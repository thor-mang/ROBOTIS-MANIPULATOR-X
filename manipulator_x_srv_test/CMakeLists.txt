cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_srv_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  manipulator_x_position_ctrl_module_msgs
  cmake_modules
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

add_executable(manipulator_x_srv_test
  src/manipulator_x_srv_test.cpp
)

add_dependencies(manipulator_x_srv_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_x_srv_test
  ${catkin_LIBRARIES}
)
