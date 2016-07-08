# ROBOTIS-MANIPULATOR-X

Required Package
- ROBOTIS-FRAMEWORK (branch : modify_torque_control)
- ROBOTIS-FRAMEWORK-msgs
- ROBOTIS-MANIPULATOR-X (branch : gripper_model)

1. For Rviz
- roslaunch manipulator_x_description manipulator_x_ctrl.launch

2. For Robot Execution
- sudo bash
- roslaunch manipulator_x_manager manipulator_x_manager.launch
- (after launch) rostopic pub -1 /robotis/torque_ctrl/set_mode_msg std_msgs/String "set"
