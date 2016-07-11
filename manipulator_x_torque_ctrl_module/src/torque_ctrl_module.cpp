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
 * torque_ctrl_module.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch
 */

#include <stdio.h>
#include "manipulator_x_torque_ctrl_module/torque_ctrl_module.h"

using namespace robotis_manipulator_x;

TorqueCtrlModule::TorqueCtrlModule()
  : control_cycle_msec_(8),
    gazebo_()
{
  enable_       = false;
  module_name_  = "torque_ctrl_module";
  control_mode_ = robotis_framework::TorqueControl;

  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["joint5"] = new robotis_framework::DynamixelState();
  result_["joint6"] = new robotis_framework::DynamixelState();
  result_["grip_joint"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;
  joint_name_to_id_["joint5"] = 5;
  joint_name_to_id_["joint6"] = 6;
  joint_name_to_id_["grip_joint"] = 7;

  joint_state_  = new TorqueCtrlJointState();
}

TorqueCtrlModule::~TorqueCtrlModule()
{
  queue_thread_.join();
}

void TorqueCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TorqueCtrlModule::queueThread, this));
}

void TorqueCtrlModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  gazebo_ = ros_node.param<bool>("gazebo", false);
  setKinematicsChain();

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_mode_msg", 5,
                                                        &TorqueCtrlModule::setModeMsgCallback, this);

  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void TorqueCtrlModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("----- Set Mode -----");

  std_msgs::String str_msg;
  str_msg.data = "torque_ctrl_module";
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void TorqueCtrlModule::setKinematicsChain()
{
  if (gazebo_ == false)
  {
    chain_.addSegment(KDL::Segment("base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.012, 0.0, 0.034)),
                                   KDL::RigidBodyInertia(0.08581,
                                                         KDL::Vector(-0.01173, 0.0, -0.01621),
                                                         KDL::RotationalInertia(0.0000136, 0.00002352, 0.0000208, 0.0, -0.00000022, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, -0.017, 0.03)),
                                   KDL::RigidBodyInertia(0.00795,
                                                         KDL::Vector(0.0, 0.017, -0.02025),
                                                         KDL::RotationalInertia(0.00000265, 0.00000105, 0.00000246, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.024, 0.0, 0.1045)),
                                   KDL::RigidBodyInertia(0.21941,
                                                         KDL::Vector(-0.01865, 0.01652, -0.04513),
                                                         KDL::RotationalInertia(0.00043395, 0.00044404, 0.00005415, 0.00000013, -0.00005129, -0.00000018)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint3",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.024)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.0, -0.01212),
                                                         KDL::RotationalInertia(0.00002580, 0.00003203, 0.00002291, 0.0, -0.00000144, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint4",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.0425, -0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09226,
                                                         KDL::Vector(-0.01321, 0.01643, 0.0),
                                                         KDL::RotationalInertia(0.00001535, 0.00002498, 0.00002865, 0.00000012, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint5",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.00000, 0.01140),
                                                         KDL::RotationalInertia(0.00002577, 0.00003200, 0.00002291, 0.0, -0.00000087, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("joint6",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.029, -0.016, 0.023)),
                                   KDL::RigidBodyInertia(0.12680,
                                                         KDL::Vector(-0.00155, 0.019, -0.02343),
                                                         KDL::RotationalInertia(0.00004897, 0.00003261, 0.00004008, -0.00000004, 0.00000006, 0.0000001)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("grip_joint",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.112, 0.016, -0.023)),
                                   KDL::RigidBodyInertia(0.13441,
                                                         KDL::Vector(-0.08534, 0, 0),
                                                         KDL::RotationalInertia(0.00002875, 0.00006646, 0.00005474, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
  }

  std::vector<double> minPositionLimit, maxPositionLimit;

  minPositionLimit.push_back(-120.0);   maxPositionLimit.push_back(120.0);
  minPositionLimit.push_back(-120.0);   maxPositionLimit.push_back(120.0);
  minPositionLimit.push_back(-120.0);   maxPositionLimit.push_back(120.0);
  minPositionLimit.push_back(-150.0);   maxPositionLimit.push_back(150.0);
  minPositionLimit.push_back(-120.0);   maxPositionLimit.push_back(120.0);
  minPositionLimit.push_back(-150.0);   maxPositionLimit.push_back(150.0);
  minPositionLimit.push_back(-120.0);   maxPositionLimit.push_back(120.0);

  dyn_param_ = new KDL::ChainDynParam(chain_, KDL::Vector(0.0, 0.0, -9.81));
}

void TorqueCtrlModule::calcGravityTerm()
{
  Eigen::VectorXd current_joint_position(MAX_JOINT_ID);
  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    current_joint_position.coeffRef(id-1) = joint_state_->curr_joint_state_[id].position_;

//    if (id == 3 || id == 5)
//      current_joint_position.coeffRef(id-1) = -1.0*current_joint_position.coeff(id-1);
  }

  KDL::JntArray kdl_current_joint_position;
  kdl_current_joint_position.data = current_joint_position;

  KDL::JntArray gravity_term(MAX_JOINT_ID);
  dyn_param_->JntToGravity(kdl_current_joint_position, gravity_term);

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
//    if (id == 3 || id == 5)
//      gravity_term(id-1) *= -1.0;

    if (id ==2)
      gravity_term(id-1) *= 1.2;


    joint_state_->gravity_state_[id] = gravity_term(id-1);
  }
}

void TorqueCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                               std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

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

    double joint_curr_position  = dxl->dxl_state_->present_position_;
    double joint_curr_velocity  = dxl->dxl_state_->present_velocity_;
    double joint_curr_effort    = dxl->dxl_state_->present_torque_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_  = joint_curr_position;
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].velocity_  = joint_curr_velocity;
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].effort_    = joint_curr_effort;
  }

  /*----- Calculate G(q) -----*/
  calcGravityTerm();

  /*----- Set Goal Joint Data -----*/
  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    joint_state_->goal_joint_state_[id].effort_ = joint_state_->gravity_state_[id];
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_torque_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].effort_;
  }
}

void TorqueCtrlModule::stop()
{
  return;
}

bool TorqueCtrlModule::isRunning()
{
  return false;
}
