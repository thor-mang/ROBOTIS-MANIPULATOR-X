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
    gazebo_(),
    gripper_(),
    module_control_(GRAVITY_COMPENSATION)
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

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;
  joint_name_to_id_["joint5"] = 5;
  joint_name_to_id_["joint6"] = 6;

  gain_path_ = ros::package::getPath("manipulator_x_torque_ctrl_module") + "/config/torque_control_pid_gain.yaml";

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

  ros_node.getParam("gazebo", gazebo_);
  ros_node.getParam("gripper", gripper_);

  setKinematicsChain();

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  get_joint_gain_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_gain",
                                                     &TorqueCtrlModule::getJointGainCallback, this);
  get_joint_pose_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_pose",
                                                     &TorqueCtrlModule::getJointPoseCallback, this);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_mode_msg", 5,
                                                        &TorqueCtrlModule::setModeMsgCallback, this);
  ros::Subscriber save_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/save_gain_msg", 5,
                                                         &TorqueCtrlModule::saveGainMsgCallback, this);
  ros::Subscriber set_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_gain_msg", 5,
                                                        &TorqueCtrlModule::setGainMsgCallback, this);
  ros::Subscriber set_joint_pose_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_joint_pose_msg", 5,
                                                              &TorqueCtrlModule::setJointPoseMsgCallback, this);
  ros::Subscriber set_wrench_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_wrench_msg", 5,
                                                          &TorqueCtrlModule::setWrenchMsgCallback, this);

  ros::Subscriber enable_joint_control_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/enable_joint_control_msg", 5,
                                                                    &TorqueCtrlModule::enableJointControlMsgCallback, this);
  ros::Subscriber enable_force_control_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/enable_force_control_msg", 5,
                                                                    &TorqueCtrlModule::enableForceControlMsgCallback, this);

  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void TorqueCtrlModule::parseGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return ;
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    YAML::Node joint_node = doc[it->first];

    p_gain_(joint_name_to_id_[it->first]-1) = joint_node["p_gain"].as<double>();
    i_gain_(joint_name_to_id_[it->first]-1) = joint_node["i_gain"].as<double>();
    d_gain_(joint_name_to_id_[it->first]-1) = joint_node["d_gain"].as<double>();
  }
}

void TorqueCtrlModule::saveGainData(const std::string &path)
{
  YAML::Emitter out;
  out << YAML::BeginMap;

  std::map<std::string, double> gain_value;

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    gain_value["p_gain"] = p_gain_(joint_name_to_id_[it->first]-1);
    gain_value["i_gain"] = i_gain_(joint_name_to_id_[it->first]-1);
    gain_value["d_gain"] = d_gain_(joint_name_to_id_[it->first]-1);

    out << YAML::Key << it->first << YAML::Value << gain_value;
  }

  out << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out.c_str();
}

void TorqueCtrlModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  //  ROS_INFO("--- Set Torque Control Mode ---");

  std_msgs::String str_msg;
  str_msg.data = "torque_ctrl_module";
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void TorqueCtrlModule::saveGainMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  //  ROS_INFO("--- Save Gain ---");

  saveGainData(gain_path_);
}

void TorqueCtrlModule::setGainMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointGain::ConstPtr& msg)
{
  goal_joint_position_ = present_joint_position_;

  //  ROS_INFO("--- Set Torque Control PID Gain ---");
  for (int it=0; it<msg->joint_name.size(); it++)
  {
    p_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->p_gain[it];
    i_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->i_gain[it];
    d_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->d_gain[it];
  }
}

void TorqueCtrlModule::setJointPoseMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  //  ROS_INFO("--- Set Desired Joint Angle ---");
  for (int it=0; it<msg->joint_name.size(); it++)
    goal_joint_position_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->position[it];
}

void TorqueCtrlModule::setWrenchMsgCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
  goal_task_wrench_(0) = msg->force.x;
  goal_task_wrench_(1) = msg->force.y;
  goal_task_wrench_(2) = msg->force.z;
  goal_task_wrench_(3) = msg->torque.x;
  goal_task_wrench_(4) = msg->torque.y;
  goal_task_wrench_(5) = msg->torque.z;
}

void TorqueCtrlModule::enableJointControlMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
    module_control_ = JOINT_CONTROL;
}

void TorqueCtrlModule::enableForceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
    module_control_ = FORCE_CONTROL;
}

bool TorqueCtrlModule::getJointGainCallback(manipulator_x_torque_ctrl_module_msgs::GetJointGain::Request &req,
                                            manipulator_x_torque_ctrl_module_msgs::GetJointGain::Response &res)
{
  if (enable_==false)
    return false;

  parseGainData( gain_path_ );

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.torque_ctrl_gain.joint_name.push_back(it->first);
    res.torque_ctrl_gain.p_gain.push_back(p_gain_(joint_name_to_id_[it->first]-1));
    res.torque_ctrl_gain.i_gain.push_back(i_gain_(joint_name_to_id_[it->first]-1));
    res.torque_ctrl_gain.d_gain.push_back(d_gain_(joint_name_to_id_[it->first]-1));
  }

  return true;
}

bool TorqueCtrlModule::getJointPoseCallback(manipulator_x_torque_ctrl_module_msgs::GetJointPose::Request &req,
                                            manipulator_x_torque_ctrl_module_msgs::GetJointPose::Response &res)
{
  if (enable_==false)
    return false;

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.torque_ctrl_joint_pose.joint_name.push_back(it->first);
    res.torque_ctrl_joint_pose.position.push_back(present_joint_position_(joint_name_to_id_[it->first]-1));
  }

  calcJacobian();

  return true;
}

void TorqueCtrlModule::calcGravityTerm()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::JntArray gravity_term(MAX_JOINT_ID);
  dyn_param_->JntToGravity(kdl_curr_joint_position, gravity_term);

  gravity_term_ = gravity_term.data;
}

void TorqueCtrlModule::calcCoriolisTerm()
{
  KDL::JntArray kdl_curr_joint_position, kdl_curr_joint_velocity;

  kdl_curr_joint_position.data = present_joint_position_;
  kdl_curr_joint_velocity.data = present_joint_velocity_;

  KDL::JntArray coriolis_term(MAX_JOINT_ID);
  dyn_param_->JntToCoriolis(kdl_curr_joint_position, kdl_curr_joint_velocity, coriolis_term);

  coriolis_term_ = coriolis_term.data;
}

void TorqueCtrlModule::calcMassTerm()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::JntSpaceInertiaMatrix mass_term(MAX_JOINT_ID);
  dyn_param_->JntToMass(kdl_curr_joint_position, mass_term);

  mass_term_ = mass_term.data;
}

void TorqueCtrlModule::calcJacobian()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::Jacobian jacobian(MAX_JOINT_ID);
  jacobian_solver_->JntToJac(kdl_curr_joint_position,jacobian);

  jacobian_ = jacobian.data;
}

void TorqueCtrlModule::setKinematicsChain()
{
  if (gazebo_ == true)
  {
    chain_.addSegment(KDL::Segment("Base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.012, 0.0, 0.034)),
                                   KDL::RigidBodyInertia(0.08581,
                                                         KDL::Vector(-0.01173, 0.0, -0.01621),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, -0.017, 0.03)),
                                   KDL::RigidBodyInertia(0.00795,
                                                         KDL::Vector(0.0, 0.017, -0.02025),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.024, 0.0, 0.1045)),
                                   KDL::RigidBodyInertia(0.21941,
                                                         KDL::Vector(-0.01865, 0.01652, -0.04513),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint3",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.024)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.0, -0.01212),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint4",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.0425, -0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09226,
                                                         KDL::Vector(-0.01321, 0.01643, 0.0),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint5",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.00000, 0.01140),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );

    if (gripper_ == true )
    {
      chain_.addSegment(KDL::Segment("Joint6",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.14103, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.26121,
                                                           KDL::Vector(-0.09906, 0.00146, -0.00021),
                                                           KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
    }
    else
    {
      chain_.addSegment(KDL::Segment("Joint6",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.02, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.005,
                                                           KDL::Vector(-0.01126, 0.0, 0.0),
                                                           KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
    }

  }
  else
  {
    chain_.addSegment(KDL::Segment("Base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.012, 0.0, 0.034)),
                                   KDL::RigidBodyInertia(0.08581,
                                                         KDL::Vector(-0.01173, 0.0, -0.01621),
                                                         KDL::RotationalInertia(0.0000136, 0.00002352, 0.0000208, 0.0, -0.00000022, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, -0.017, 0.03)),
                                   KDL::RigidBodyInertia(0.00795,
                                                         KDL::Vector(0.0, 0.017, -0.02025),
                                                         KDL::RotationalInertia(0.00000265, 0.00000105, 0.00000246, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.024, 0.0, 0.1045)),
                                   KDL::RigidBodyInertia(0.21941,
                                                         KDL::Vector(-0.01865, 0.01652, -0.04513),
                                                         KDL::RotationalInertia(0.00043395, 0.00044404, 0.00005415, 0.00000013, -0.00005129, -0.00000018)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint3",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.024)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.0, -0.01212),
                                                         KDL::RotationalInertia(0.00002580, 0.00003203, 0.00002291, 0.0, -0.00000144, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint4",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.0425, -0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09226,
                                                         KDL::Vector(-0.01321, 0.01643, 0.0),
                                                         KDL::RotationalInertia(0.00001535, 0.00002498, 0.00002865, 0.00000012, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint5",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.062, 0.017, 0.0)),
                                   KDL::RigidBodyInertia(0.09746,
                                                         KDL::Vector(-0.01902, 0.00000, 0.01140),
                                                         KDL::RotationalInertia(0.00002577, 0.00003200, 0.00002291, 0.0, -0.00000087, 0.0)
                                                         )
                                   )
                      );
    if (gripper_ == true)
    {
      chain_.addSegment(KDL::Segment("Joint6",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.14103, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.26121,
                                                           KDL::Vector(-0.09906, 0.00146, -0.00021),
                                                           KDL::RotationalInertia(0.00019, 0.00022, 0.00029, 0.00001, 0.0, 0.0)
                                                           )
                                     )
                        );
    }
    else
    {
      chain_.addSegment(KDL::Segment("Joint6",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.02, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.005,
                                                           KDL::Vector(-0.01126, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.00000016, 0.00000021, 0.00000021, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
    }

    jacobian_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
  }

  dyn_param_ = new KDL::ChainDynParam(chain_, KDL::Vector(0.0, 0.0, -9.81));
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

    present_joint_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;
  }

  /*----- Calculate Gravity Term -----*/
  calcGravityTerm();
  //  calcCoriolisTerm();
  //  calcMassTerm();

  /*----- Set Goal Torque Data -----*/
  if (module_control_ == GRAVITY_COMPENSATION)
  {
    goal_joint_effort_ = gravity_term_;
  }
  else if (module_control_ == JOINT_CONTROL)
  {
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


  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_torque_ = goal_joint_effort_(joint_name_to_id_[joint_name]-1);
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
