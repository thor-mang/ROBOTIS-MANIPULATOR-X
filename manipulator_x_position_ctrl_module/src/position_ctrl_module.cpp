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
  : control_cycle_msec_(8)
{
  enable_ = false; //motion_module.h
  module_name_ = "manipulator_x4_position_ctrl";
  control_mode_ = robotis_framework::PositionControl;

  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["grip_joint"] = new robotis_framework::DynamixelState();
}

ManipulatorX4PositionCtrlModule::~ManipulatorX4PositionCtrlModule()
{
  queue_thread_.join();
}

void ManipulatorX4PositionCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ManipulatorX4PositionCtrlModule::queueThread,this));
}

void ManipulatorX4PositionCtrlModule::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue callback_queue;

  nh.setCallbackQueue(&callback_queue);

  joint1_position_control_pub_ = nh.advertise<std_msgs::Float64>("/robotis_manipulator_x/joint1_position/command", 1, true);
  joint2_position_control_pub_ = nh.advertise<std_msgs::Float64>("/robotis_manipulator_x/joint2_position/command", 1, true);
  joint3_position_control_pub_ = nh.advertise<std_msgs::Float64>("/robotis_manipulator_x/joint3_position/command", 1, true);
  joint4_position_control_pub_ = nh.advertise<std_msgs::Float64>("/robotis_manipulator_x/joint4_position/command", 1, true);
  grip_joint_position_control_pub_ = nh.advertise<std_msgs::Float64>("/robotis_manipulator_x/grip_joint_position/command", 1, true);

  while (nh.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void ManipulatorX4PositionCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;


}

void ManipulatorX4PositionCtrlModule::stop()
{
  return;
}

bool ManipulatorX4PositionCtrlModule::isRunning()
{
  return false;
}
