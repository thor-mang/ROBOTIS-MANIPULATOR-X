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
 * position_ctrl_module.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch
 */

#include <stdio.h>
#include "manipulator_x_position_ctrl_module/position_ctrl_module.h"

using namespace robotis_manipulator_x;

PositionCtrlModule::PositionCtrlModule()
  : control_cycle_msec_(8)
{
  enable_       = false;
  module_name_  = "position_ctrl_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["joint5"] = new robotis_framework::DynamixelState();
  result_["joint6"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;
  joint_name_to_id_["joint5"] = 5;
  joint_name_to_id_["joint6"] = 6;

  //gain_path_ = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/torque_control_pid_gain.yaml";
/*
  p_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  i_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  d_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_ID);

  error_        = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  error_prior_  = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  integral_     = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  derivative_   = Eigen::VectorXd::Zero(MAX_JOINT_ID);

  present_joint_position_  = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  present_joint_velocity_  = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  present_joint_effort_    = Eigen::VectorXd::Zero(MAX_JOINT_ID);

  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  goal_joint_velocity_      = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  goal_joint_acceleration_  = Eigen::VectorXd::Zero(MAX_JOINT_ID);
  goal_joint_effort_        = Eigen::VectorXd::Zero(MAX_JOINT_ID);

  goal_task_wrench_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
*/
}

PositionCtrlModule::~PositionCtrlModule()
{
  queue_thread_.join();
}

void PositionCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&PositionCtrlModule::queueThread, this));
}

void PositionCtrlModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  //ros_node.getParam("gazebo", gazebo_);
  //ros_node.getParam("gripper", gripper_);

  //setKinematicsChain();

  /* publish topics */
/*
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_joint_position_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_position", 1);

  get_joint_gain_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_gain",
                                                     &PositionCtrlModule::getJointGainCallback, this);
  get_joint_pose_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_pose",
                                                     &PositionCtrlModule::getJointPoseCallback, this);
*/
  /* subscribe topics */
/*
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_mode_msg", 5,
                                                        &PositionCtrlModule::setModeMsgCallback, this);
  ros::Subscriber save_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/save_gain_msg", 5,
                                                         &PositionCtrlModule::saveGainMsgCallback, this);
  ros::Subscriber set_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_gain_msg", 5,
                                                        &PositionCtrlModule::setGainMsgCallback, this);
  ros::Subscriber set_joint_pose_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_joint_pose_msg", 5,
                                                              &PositionCtrlModule::setJointPoseMsgCallback, this);
  ros::Subscriber set_wrench_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_wrench_msg", 5,
                                                          &PositionCtrlModule::setWrenchMsgCallback, this);

  ros::Subscriber enable_joint_control_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/enable_joint_control_msg", 5,
                                                                    &PositionCtrlModule::enableJointControlMsgCallback, this);
  ros::Subscriber enable_force_control_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/enable_force_control_msg", 5,
                                                                    &PositionCtrlModule::enableForceControlMsgCallback, this);
*/
  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void PositionCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                               std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

//  ros::Time now = ros::Time::now();

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
/*
    present_joint_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;
*/
  }
/*
  if (initialize_goal_value_==false)
  {
    goal_joint_position_ = present_joint_position_;
    goal_joint_velocity_ = present_joint_velocity_;
    goal_joint_effort_ = present_joint_effort_;

    initialize_goal_value_ = true;
  }
*/
  /*----- Calculate Gravity Term -----*/
  //  calcGravityTerm();
  //  calcCoriolisTerm();
  //  calcMassTerm();

  /*----- Set Goal Torque Data -----*/
/*
  if (module_control_ == GRAVITY_COMPENSATION)
  {
    goal_joint_effort_ = gravity_term_;
  }
  else if (module_control_ == JOINT_CONTROL)
  {
    if (is_moving_ == true)
    {
      for (int index=0; index<MAX_JOINT_ID; index++)
        goal_joint_position_(index) = goal_joint_tra_(cnt_,index);

//      PRINT_MAT(goal_joint_position_);

      cnt_++;
    }

//    ROS_INFO("present_position[6] : %f", present_joint_position_(5));
//    ROS_INFO("goal_position[6] : %f", goal_joint_position_(5));

    // PID Control with Gravity Compensation
    for (int it=0; it<MAX_JOINT_ID; it++)
    {
      error_(it) = goal_joint_position_(it)-present_joint_position_(it);
      integral_(it) = integral_(it) + (error_(it)*ITERATION_TIME);
      derivative_(it) = present_joint_velocity_(it); //(error_(it)-error_prior_(it))/ITERATION_TIME;

      goal_joint_effort_(it)=
          p_gain_(it)*error_(it) +
          i_gain_(it)*integral_(it) +
          d_gain_(it)*derivative_(it) +
          gravity_term_(it);

      error_prior_(it) = error_(it);
    }
  }
  else if (module_control_ == FORCE_CONTROL)
  {
    calcJacobian();
    goal_joint_effort_ = jacobian_.transpose()*goal_task_wrench_ + gravity_term_;
  }
*/
/*
  sensor_msgs::JointState goal_joint_position_msg;
*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
/*
    result_[joint_name]->goal_torque_ = goal_joint_effort_(joint_name_to_id_[joint_name]-1);
*/

/*
    goal_joint_position_msg.name.push_back(joint_name);
    goal_joint_position_msg.position.push_back(goal_joint_position_(joint_name_to_id_[joint_name]-1));
    goal_joint_position_msg.header.stamp = ros::Time::now();
*/
  }
/*
  goal_joint_position_pub_.publish(goal_joint_position_msg);
*/
//  ros::Duration dur = ros::Time::now() - now;
//  double msec = dur.nsec * 0.000001;

//  ROS_INFO_STREAM("Process duration  : " << msec);
/*
  if (module_control_ == JOINT_CONTROL)
  {
    if (is_moving_ == true && cnt_ >= all_time_steps_)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;
      cnt_ = 0;
    }
  }
*/
}

void PositionCtrlModule::stop()
{
  return;
}

bool PositionCtrlModule::isRunning()
{
  return false;
  //return is_moving_;
}
/*
void PositionCtrlModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.type = type;
    status_msg.module_name = "PositionCtrl";
    status_msg.status_msg = msg;

    status_msg_pub_.publish(status_msg);
}
*/
