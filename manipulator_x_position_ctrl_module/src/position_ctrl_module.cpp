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

/* Author: Taehoon Lim (Darby) */

#include "manipulator_x_position_ctrl_module/position_ctrl_module.h"

using namespace manipulator_x4_position_ctrl_module;

ManipulatorX4PositionCtrlModule::ManipulatorX4PositionCtrlModule()
  : control_cycle_msec_(8),
    using_gazebo_()
{
  enable_ = false; //motion_module.h
  module_name_ = "manipulator_x4_position_ctrl";
  control_mode_ = robotis_framework::PositionControl;

  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;

  joint_present_position_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  joint_present_velocity_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  joint_present_current_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  joint_goal_position_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
}

ManipulatorX4PositionCtrlModule::~ManipulatorX4PositionCtrlModule()
{
  queue_thread_.join();
}

void ManipulatorX4PositionCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle nh;
  nh.getParam("gazebo", using_gazebo_);

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ManipulatorX4PositionCtrlModule::queueThread, this));
}

void ManipulatorX4PositionCtrlModule::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue callback_queue;

  nh.setCallbackQueue(&callback_queue);

  status_msg_pub_  = nh.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 10);
  joint_present_position_server_ = nh.advertiseService("/robotis/manipulator_x4_position_ctrl/joint_present_position",
                                                       &ManipulatorX4PositionCtrlModule::getJointPresentPositionCallback, this);

  set_mode_msg_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/set_mode_msg", 10,
                                   &ManipulatorX4PositionCtrlModule::setModeMsgCallback, this);

  joint_goal_position_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/send_goal_position", 10,
                                          &ManipulatorX4PositionCtrlModule::setJointGoalPositionCallback, this);

  while (nh.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void ManipulatorX4PositionCtrlModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Manipulator-X4 Poition Control Module");
}

bool ManipulatorX4PositionCtrlModule::getJointPresentPositionCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
                                                                      manipulator_x_position_ctrl_module_msgs::GetJointPose::Response &res)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Joint Present Position");

  for (std::map<std::string, uint8_t>::iterator joint_iter = joint_name_to_id_.begin();
       joint_iter != joint_name_to_id_.end(); joint_iter++)
  {
    res.position_ctrl_joint_pose.joint_name.push_back(joint_iter->first);
    res.position_ctrl_joint_pose.position.push_back(joint_present_position_(joint_name_to_id_[joint_iter->first]-1));
  }

  return true;
}

void ManipulatorX4PositionCtrlModule::setJointGoalPositionCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Goal Position");

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    joint_goal_position_(joint_name_to_id_[joint_name]-1) = msg->position[joint_name_to_id_[joint_name]-1];
  }
}

void ManipulatorX4PositionCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
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

    joint_present_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    joint_present_velocity_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    joint_present_current_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;
  }

  /* Set Joint Pose */
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = joint_goal_position_(joint_name_to_id_[joint_name]-1);
  }
}

void ManipulatorX4PositionCtrlModule::stop()
{
  return;
}

bool ManipulatorX4PositionCtrlModule::isRunning()
{
  return false;
}

void ManipulatorX4PositionCtrlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Manipulator_X4_Position_ctrl";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
