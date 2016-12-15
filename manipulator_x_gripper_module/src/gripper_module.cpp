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
 * gripper_module.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch, Darby Lim
 */

#include <stdio.h>
#include "manipulator_x_gripper_module/gripper_module.h"

using namespace manipulator_x4_gripper_module;

ManipulatorX4GripperModule::ManipulatorX4GripperModule()
  :control_cycle_msec_(8),
   using_gazebo_(),
   is_moving_(false)
{
  enable_ = false;
  module_name_ = "gripper_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["grip_joint"] = new robotis_framework::DynamixelState();

  joint_id_["grip_joint"] = 1;

  joint_present_position_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  joint_present_velocity_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  joint_present_current_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  joint_goal_position_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
}

ManipulatorX4GripperModule::~ManipulatorX4GripperModule()
{
  queue_thread_.join();
}

void ManipulatorX4GripperModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle nh;
  nh.getParam("gazebo", using_gazebo_);

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ManipulatorX4GripperModule::queueThread, this));
}

void ManipulatorX4GripperModule::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue callback_queue;

  nh.setCallbackQueue(&callback_queue);

  status_msg_pub_ = nh.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 10);
  set_gripper_module_msg_sub_ = nh.subscribe("/robotis/manipulator_x4_gripper/set_module_msg", 10,
                                   &ManipulatorX4GripperModule::setModuleMsgCallback, this);

  gripper_goal_position_sub_ = nh.subscribe("/robotis/manipulator_x4_gripper/send_goal_position", 10,
                                          &ManipulatorX4GripperModule::setGripperPositionCallback, this);

  while (nh.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void ManipulatorX4GripperModule::setModuleMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Manipulator-X4 Gripper Module");
}

void ManipulatorX4GripperModule::setGripperPositionCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set gripper module");
    return;
  }

  if (is_moving_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Gripper Goal Position");

    Eigen::VectorXd initial_position = joint_goal_position_;
    Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);

    move_time_ = 1.0;

    for (int it=0; it<MAX_GRIPPER_NUM; it++)
    {
      target_position(it) = msg->data;
    }

    calculateGoalJointTrajectory(initial_position, target_position);
  }
  else
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
  }
}

void ManipulatorX4GripperModule::calculateGoalJointTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  /* set movement time */
  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  joint_goal_trajectory_.resize(all_time_steps_, MAX_GRIPPER_NUM);

  /* calculate joint trajectory */
  for (int index = 0; index < MAX_GRIPPER_NUM; index++)
  {
    double init_position_value = initial_position(index);
    double target_position_value = target_position(index);

    Eigen::MatrixXd trajectory =
        robotis_framework::calcMinimumJerkTra(init_position_value, 0.0, 0.0,
                                              target_position_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    // Block of size (p,q), starting at (i,j)
    // block(i,j,p,q)
    joint_goal_trajectory_.block(0, index, all_time_steps_, 1) = trajectory;
  }

  step_cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void ManipulatorX4GripperModule::process(std::map<std::__cxx11::string, robotis_framework::Dynamixel *> dxls, std::map<std::__cxx11::string, double> sensors)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  /* Get Joint State */
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    joint_present_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    joint_present_velocity_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    joint_present_current_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;

    joint_goal_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->goal_position_;
  }

  /* Set Joint Pose */
  if (is_moving_ == true)
  {
    for (int index = 0; index < MAX_GRIPPER_NUM; index++)
    {
      joint_goal_position_(index) = joint_goal_trajectory_(step_cnt_, index);
    }
    step_cnt_++;
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = joint_goal_position_(joint_id_[joint_name]-1);
  }

  if (is_moving_ == true)
  {
    if (step_cnt_ >= all_time_steps_)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;
      step_cnt_ = 0;
    }
  }
}

void ManipulatorX4GripperModule::stop()
{
  return;
}

bool ManipulatorX4GripperModule::isRunning()
{
  return is_moving_;
}

void ManipulatorX4GripperModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Manipulator_X4_Gripper_Position_ctrl";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
