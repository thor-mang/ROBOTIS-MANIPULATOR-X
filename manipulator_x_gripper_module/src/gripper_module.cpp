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
 *      Author: sch
 */

#include <stdio.h>
#include "manipulator_x_gripper_module/gripper_module.h"

using namespace robotis_manipulator_x;

GripperModule::GripperModule()
  : control_cycle_msec_(8),
    using_gazebo_(),
    using_gripper_(),
    is_moving_(false)
{
  enable_       = false;
  module_name_  = "gripper_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["grip_joint"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["grip_joint"] = 1;

  present_joint_position_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  present_joint_velocity_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  present_joint_effort_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  goal_joint_position_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
}

GripperModule::~GripperModule()
{
  queue_thread_.join();
}

void GripperModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle ros_node;
  ros_node.getParam("gazebo", using_gazebo_);
  ros_node.getParam("gripper", using_gripper_);

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&GripperModule::queueThread, this));
}

void GripperModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/gripper/set_mode_msg", 5,
                                                        &GripperModule::setModeMsgCallback, this);
  ros::Subscriber set_gripper_pose_msg_sub = ros_node.subscribe("/robotis/gripper/set_gripper_pose_msg", 5,
                                                                &GripperModule::setGripperPoseMsgCallback, this);

  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void GripperModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Gripper Module");
  std_msgs::String str_msg;
  str_msg.data = "gripper_module";
  set_ctrl_module_pub_.publish(str_msg);
}

void GripperModule::setGripperPoseMsgCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if (is_moving_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Goal Joint Values");

    Eigen::VectorXd initial_position = goal_joint_position_;
    Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);

    mov_time_ = 1.0;

    for (int it=0; it<MAX_GRIPPER_NUM; it++)
      target_position(it) = msg->data;

    calcGoalJointTra(initial_position, target_position);
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Previous Task is Alive");
}

void GripperModule::calcGoalJointTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  /* set movement time */
  all_time_steps_ = int(floor((mov_time_ / ITERATION_TIME ) + 1.0));
  mov_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;
  goal_joint_tra_.resize(all_time_steps_ , MAX_GRIPPER_NUM);

  /* calculate joint trajectory */
  for (int index=0; index<MAX_GRIPPER_NUM; index++)
  {
    double ini_value = initial_position(index);
    double tar_value = target_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value, 0.0, 0.0,
                                              ITERATION_TIME, mov_time_);

    goal_joint_tra_.block(0, index, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void GripperModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                 std::map<std::string, double> sensors)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Gripper Control Module");
    return;
  }

  /*----- Get Joint State -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    present_joint_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;

    goal_joint_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->goal_position_;
  }

  /*----- Set Goal Data -----*/
  if (is_moving_ == true)
  {
    for (int index=0; index<MAX_GRIPPER_NUM; index++)
      goal_joint_position_(index) = goal_joint_tra_(cnt_,index);

    cnt_++;
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]-1);
  }

  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;
      cnt_ = 0;
    }
  }
}

void GripperModule::stop()
{
  return;
}

bool GripperModule::isRunning()
{
  return is_moving_;
}

void GripperModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Gripper";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
