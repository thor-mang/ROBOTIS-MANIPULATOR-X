# ROBOTIS-MANIPULATOR-X

1.
- roslaunch manipulator_x_description manipulator_x_ctrl.launch

2.
- sudo bash
- roslaunch manipulator_x_manager manipulator_x_manager.launch
- (after launch) rostopic pub -1 /robotis/torque_ctrl/set_mode_msg std_msgs/String "set"
