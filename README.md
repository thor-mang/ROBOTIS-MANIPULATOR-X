# ROBOTIS-MANIPULATOR-X for Position Control

### 1. Required Package

- ROBOTIS-MANIPULATOR-X (branch : master / position_control)   
- ROBOTIS-Framework (branch : develop)  
- ROBOTIS-Framework-msgs  
- qt_ros (https://github.com/stonier/qt_ros, branch : indigo)  
- gazebo_ros_pkgs (https://github.com/ros-simulation/gazebo_ros_pkgs, branch : kinetic)  

### 2. Execution

#### 2.1. run manager  
```  
$ sudo bash  
# roslaunch manipulator_x_manager manipulator_x_manager.launch  
```  

#### 2.2. run gui  
```  
$ rosrun manipulator_x_position_ctrl_gui manipulator_x_position_ctrl_gui  
```  

#### 2.3. run rviz  
```  
$roslaunch manipulator_x_description manipulator_x_ctrl.launch  
```  
