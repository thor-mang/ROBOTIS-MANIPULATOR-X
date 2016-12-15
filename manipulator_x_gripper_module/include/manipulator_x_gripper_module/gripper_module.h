/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * gripper_module.h
 *
 *  Created on: Jul 6, 2016
 *      Author: sch, Darby Lim
 */

#ifndef MOTION_MODULE_MANIPULATOR_X4_GRIPPER_MODULE_H
#define MOTION_MODULE_MANIPULATOR_X4_GRIPPER_MODULE_H

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace manipulator_x4_gripper_module
{
#define MAX_GRIPPER_NUM (1)
#define ITERATION_TIME  (0.008)

class ManipulatorX4GripperModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<ManipulatorX4GripperModule>
{
private:
  int control_cycle_msec_;
  boost::thread queue_thread_;

  ros::Publisher status_msg_pub_;

  ros::Subscriber set_gripper_module_msg_sub_;
  ros::Subscriber gripper_goal_position_sub_;

  std::map<std::string, uint8_t> joint_id_;

  bool using_gazebo_;
  bool is_moving_;
  double move_time_;
  int all_time_steps_, step_cnt_;

  Eigen::VectorXd joint_present_position_;
  Eigen::VectorXd joint_present_velocity_;
  Eigen::VectorXd joint_present_current_;
  Eigen::VectorXd joint_goal_position_;

  Eigen::MatrixXd joint_goal_trajectory_;

  void queueThread();

  void setModuleMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setGripperPositionCallback(const std_msgs::Float64::ConstPtr& msg);

  void calculateGoalJointTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
  void publishStatusMsg(unsigned int type, std::string msg);

public:
  ManipulatorX4GripperModule();
  virtual ~ManipulatorX4GripperModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};
}

#endif // MOTION_MODULE_MANIPULATOR_X4_GRIPPER_MODULE_H
