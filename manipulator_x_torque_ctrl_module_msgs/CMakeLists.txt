cmake_minimum_required(VERSION 2.8.3)
project(manipulator_x_torque_ctrl_module_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}    
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  JointGain.msg
  JointPose.msg
  KinematicsGain.msg
  KinematicsPose.msg
)

add_service_files(
  FILES
  GetJointGain.srv
  GetJointPose.srv
  GetKinematicsGain.srv
  GetKinematicsPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
