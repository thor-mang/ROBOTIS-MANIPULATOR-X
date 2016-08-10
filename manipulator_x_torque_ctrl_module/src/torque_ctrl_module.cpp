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
    using_gazebo_(),
    using_gripper_(),
    module_control_(GRAVITY_COMPENSATION),
    is_moving_(false),
    initialize_goal_value_(false)
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
  force_gain_path_ = ros::package::getPath("manipulator_x_torque_ctrl_module") + "/config/torque_control_force_gain.yaml";

  p_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  i_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  d_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  error_        = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  error_prior_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  integral_     = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  derivative_   = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  present_joint_position_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  present_joint_velocity_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  present_joint_effort_    = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_joint_velocity_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_joint_acceleration_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_joint_effort_        = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  force_joint_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  force_p_gain_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  force_i_gain_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  force_d_gain_ = Eigen::VectorXd::Zero(TASK_DEMENSION);

  u_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  force_target_position_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  force_current_position_ = Eigen::VectorXd::Zero(TASK_DEMENSION);

  goal_pose_error_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  goal_pose_error_prior_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  goal_task_wrench_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  goal_task_wrench_derivative_ = Eigen::VectorXd::Zero(TASK_DEMENSION);
  goal_joint_velocity_damping_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
}

TorqueCtrlModule::~TorqueCtrlModule()
{
  queue_thread_.join();
}

void TorqueCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle ros_node;
  ros_node.getParam("gazebo", using_gazebo_);
  ros_node.getParam("gripper", using_gripper_);

  setKinematicsChain();

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TorqueCtrlModule::queueThread, this));
}

void TorqueCtrlModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_joint_states_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/torque_ctrl/goal_joint_states", 1);

  get_joint_gain_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_gain",
                                                     &TorqueCtrlModule::getJointGainCallback, this);
  get_joint_pose_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_joint_pose",
                                                     &TorqueCtrlModule::getJointPoseCallback, this);
  get_kinematics_pose_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_kinematics_pose",
                                                          &TorqueCtrlModule::getKinematicsPoseCallback, this);
  get_force_gain_server_ = ros_node.advertiseService("/robotis/torque_ctrl/get_kinematics_gain",
                                                     &TorqueCtrlModule::getKinematicsGainCallback, this);

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
  ros::Subscriber set_kinematics_pose_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_kinematics_pose_msg", 5,
                                                                   &TorqueCtrlModule::setKinematicsPoseMsgCallback, this);
  ros::Subscriber set_kinematics_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/set_kinematics_gain_msg", 5,
                                                                   &TorqueCtrlModule::setKinematicsGainMsgCallback, this);
  ros::Subscriber save_force_gain_msg_sub = ros_node.subscribe("/robotis/torque_ctrl/save_foece_gain_msg", 5,
                                                               &TorqueCtrlModule::saveForceGainMsgCallback, this);

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

Eigen::MatrixXd TorqueCtrlModule::parseGainData(const std::string &path)
{
  Eigen::MatrixXd pid_gain = Eigen::MatrixXd::Zero(MAX_JOINT_NUM,3);

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return pid_gain;
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    YAML::Node joint_node = doc[it->first];

    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,0) = joint_node["p_gain"].as<double>();
    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,1) = joint_node["i_gain"].as<double>();
    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,2) = joint_node["d_gain"].as<double>();
  }

  return pid_gain;
}

Eigen::MatrixXd TorqueCtrlModule::parseForceGainData(const std::string &path)
{
  Eigen::MatrixXd pid_joint_gain = Eigen::MatrixXd::Zero(TASK_DEMENSION,4);

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return pid_joint_gain;
  }

  YAML::Node force_p_gain_node, force_d_gain_node, force_joint_gain_node;

  force_p_gain_node = doc["force_p_gain"];
  force_d_gain_node = doc["force_d_gain"];
  force_joint_gain_node = doc["force_joint_gain"];

  for (int it=0; it<TASK_DEMENSION; it++)
  {
    pid_joint_gain.coeffRef(it,0) = force_p_gain_node[it].as<double>();
    pid_joint_gain.coeffRef(it,2) = force_d_gain_node[it].as<double>();
    pid_joint_gain.coeffRef(it,3) = force_joint_gain_node[it].as<double>();
  }

  return pid_joint_gain;
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

void TorqueCtrlModule::saveForceGainData(const std::string &path)
{
  YAML::Emitter out;
  out << YAML::BeginMap;

  std::map<int, double> p_gain, d_gain, joint_gain;

  for(int it=0; it<TASK_DEMENSION; it++)
  {
    p_gain[it] = force_p_gain_(it);
    d_gain[it] = force_d_gain_(it);
    joint_gain[it] = force_joint_gain_(it);
  }

  out << YAML::Key << "force_p_gain" << YAML::Value << p_gain;
  out << YAML::Key << "force_d_gain" << YAML::Value << d_gain;
  out << YAML::Key << "force_joint_gain" << YAML::Value << joint_gain;

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

void TorqueCtrlModule::saveForceGainMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  //  ROS_INFO("--- Save Gain ---");
  saveForceGainData(force_gain_path_);
}

void TorqueCtrlModule::setGainMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointGain::ConstPtr& msg)
{
//  goal_joint_position_ = present_joint_position_;

  //  ROS_INFO("--- Set Torque Control PID Gain ---");
  for (int it=0; it<msg->joint_name.size(); it++)
  {
    p_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->p_gain[it];
    i_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->i_gain[it];
    d_gain_(joint_name_to_id_[msg->joint_name[it]]-1) = msg->d_gain[it];
  }
}

void TorqueCtrlModule::setKinematicsGainMsgCallback(const manipulator_x_torque_ctrl_module_msgs::KinematicsGain::ConstPtr& msg)
{
  for (int it=0; it<msg->pose_value.size(); it++)
  {
    force_p_gain_(msg->pose_value[it]) = msg->p_gain[it];
    force_d_gain_(msg->pose_value[it]) = msg->d_gain[it];
  }
}

void TorqueCtrlModule::setJointPoseMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  Eigen::VectorXd initial_joint_position = goal_joint_position_;
  Eigen::VectorXd target_joint_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  mov_time_ = msg->mov_time;

  //  ROS_INFO("--- Set Desired Joint Angle ---");
  for (int it=0; it<msg->joint_name.size(); it++)
    target_joint_position(joint_name_to_id_[msg->joint_name[it]]-1) = msg->position[it];

  calcGoalJointTra(initial_joint_position, target_joint_position);
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

void TorqueCtrlModule::setKinematicsPoseMsgCallback(const manipulator_x_torque_ctrl_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Torque Control Module");
    return;
  }

  if (module_control_ == FORCE_CONTROL)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Goal Kinematics Pose");

    Eigen::VectorXd initial_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd target_position = Eigen::VectorXd::Zero(3);

    mov_time_ = msg->mov_time;

    calcForwardKinematics();

    initial_position(0) = present_kinematics_pose_.position.x;
    initial_position(1) = present_kinematics_pose_.position.y;
    initial_position(2) = present_kinematics_pose_.position.z;

    target_position(0) = msg->pose.position.x;
    target_position(1) = msg->pose.position.y;
    target_position(2) = msg->pose.position.z;

    initial_orientation_.x() = present_kinematics_pose_.orientation.x;
    initial_orientation_.y() = present_kinematics_pose_.orientation.y;
    initial_orientation_.z() = present_kinematics_pose_.orientation.z;
    initial_orientation_.w() = present_kinematics_pose_.orientation.w;

    target_orientation_.x() = msg->pose.orientation.x;
    target_orientation_.y() = msg->pose.orientation.y;
    target_orientation_.z() = msg->pose.orientation.z;
    target_orientation_.w() = msg->pose.orientation.w;

    calcGoalTaskTra(initial_position, target_position);
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Check Enable Force Control");
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

  force_target_position_ = force_current_position_;
  force_target_orientation_ = force_current_orientation_;
}

bool TorqueCtrlModule::getJointGainCallback(manipulator_x_torque_ctrl_module_msgs::GetJointGain::Request &req,
                                            manipulator_x_torque_ctrl_module_msgs::GetJointGain::Response &res)
{
  if (enable_==false)
    return false;

  Eigen::MatrixXd pid_gain = parseGainData(gain_path_);

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.torque_ctrl_gain.joint_name.push_back(it->first);
    res.torque_ctrl_gain.p_gain.push_back(pid_gain.coeff(joint_name_to_id_[it->first]-1,0));
    res.torque_ctrl_gain.i_gain.push_back(pid_gain(joint_name_to_id_[it->first]-1,1));
    res.torque_ctrl_gain.d_gain.push_back(pid_gain(joint_name_to_id_[it->first]-1,2));
  }

  return true;
}

bool TorqueCtrlModule::getKinematicsGainCallback(manipulator_x_torque_ctrl_module_msgs::GetKinematicsGain::Request &req,
                                                 manipulator_x_torque_ctrl_module_msgs::GetKinematicsGain::Response &res)
{
  if (enable_==false)
    return false;

  Eigen::MatrixXd pid_joint_gain = parseForceGainData(force_gain_path_);

  for (int it=0; it<TASK_DEMENSION; it++)
  {
    res.torque_ctrl_force_gain.pose_value.push_back(it);
    res.torque_ctrl_force_gain.p_gain.push_back(pid_joint_gain.coeff(it,0));
    res.torque_ctrl_force_gain.d_gain.push_back(pid_joint_gain.coeff(it,2));
    res.torque_ctrl_force_gain.joint_gain.push_back(pid_joint_gain.coeff(it,3));

    force_joint_gain_(it) = pid_joint_gain.coeff(it,3);
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

bool TorqueCtrlModule::getKinematicsPoseCallback(manipulator_x_torque_ctrl_module_msgs::GetKinematicsPose::Request &req,
                                                 manipulator_x_torque_ctrl_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Torque Control Module");
    return false;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Present Kinematics Pose");

  calcForwardKinematics();

  res.torque_ctrl_kinematics_pose.group_name = "arm";
  res.torque_ctrl_kinematics_pose.pose = present_kinematics_pose_;

  return true;
}

void TorqueCtrlModule::calcGravityTerm()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::JntArray gravity_term(MAX_JOINT_NUM);
  dyn_param_->JntToGravity(kdl_curr_joint_position, gravity_term);

  gravity_term_ = gravity_term.data;
}

void TorqueCtrlModule::calcCoriolisTerm()
{
  KDL::JntArray kdl_curr_joint_position, kdl_curr_joint_velocity;

  kdl_curr_joint_position.data = present_joint_position_;
  kdl_curr_joint_velocity.data = present_joint_velocity_;

  KDL::JntArray coriolis_term(MAX_JOINT_NUM);
  dyn_param_->JntToCoriolis(kdl_curr_joint_position, kdl_curr_joint_velocity, coriolis_term);

  coriolis_term_ = coriolis_term.data;
}

void TorqueCtrlModule::calcMassTerm()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::JntSpaceInertiaMatrix mass_term(MAX_JOINT_NUM);
  dyn_param_->JntToMass(kdl_curr_joint_position, mass_term);

  mass_term_ = mass_term.data;
}

void TorqueCtrlModule::calcJacobian()
{
  KDL::JntArray kdl_curr_joint_position;
  kdl_curr_joint_position.data = present_joint_position_;

  KDL::Jacobian jacobian(MAX_JOINT_NUM);
  jacobian_solver_->JntToJac(kdl_curr_joint_position,jacobian);

  jacobian_ = jacobian.data;
}

void TorqueCtrlModule::calcForwardKinematics()
{
  KDL::JntArray kdl_joint_position;
  kdl_joint_position.data = present_joint_position_;

  KDL::Frame kdl_kinematics_pose;
  forward_kinematics_solver_->JntToCart(kdl_joint_position, kdl_kinematics_pose);

  present_kinematics_pose_.position.x = kdl_kinematics_pose.p.x();
  present_kinematics_pose_.position.y = kdl_kinematics_pose.p.y();
  present_kinematics_pose_.position.z = kdl_kinematics_pose.p.z();

  kdl_kinematics_pose.M.GetQuaternion(present_kinematics_pose_.orientation.x,
                                      present_kinematics_pose_.orientation.y,
                                      present_kinematics_pose_.orientation.z,
                                      present_kinematics_pose_.orientation.w);
}

void TorqueCtrlModule::calcGoalJointTra(Eigen::VectorXd initial_joint_position, Eigen::VectorXd target_joint_position)
{
  if( enable_ == false )
    return;

  /* set movement time */
  all_time_steps_ = int(floor((mov_time_ / ITERATION_TIME ) + 1.0));
  mov_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  all_time_steps_ = int(mov_time_ / ITERATION_TIME) + 1;
  goal_joint_tra_.resize(all_time_steps_ , MAX_JOINT_NUM);
  goal_joint_tra_vel_.resize(all_time_steps_ , MAX_JOINT_NUM);
  goal_joint_tra_accel_.resize(all_time_steps_ , MAX_JOINT_NUM);

  /* calculate joint trajectory */
  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    double ini_value = initial_joint_position(index);
    double tar_value = target_joint_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTraPlus(ini_value, 0.0, 0.0,
                                                  tar_value, 0.0, 0.0,
                                                  ITERATION_TIME, mov_time_);

    goal_joint_tra_.block(0, index, all_time_steps_, 1) = tra.block(0, 0, all_time_steps_, 1);
    goal_joint_tra_vel_.block(0, index, all_time_steps_, 1) = tra.block(0, 1, all_time_steps_, 1);
    goal_joint_tra_accel_.block(0, index, all_time_steps_, 1) = tra.block(0, 2, all_time_steps_, 1);
  }

  cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void TorqueCtrlModule::calcGoalTaskTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Torque Control Module");
    return;
  }

  /* set movement time */
  all_time_steps_ = int(floor((mov_time_ / ITERATION_TIME) + 1.0));
  mov_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;
  goal_task_tra_.resize(all_time_steps_, 3);

  /* calculate task position trajectory */
  for (int index=0; index<3; index++)
  {
    double ini_value = initial_position(index);
    double tar_value = target_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value, 0.0, 0.0,
                                              ITERATION_TIME, mov_time_);

    goal_task_tra_.block(0, index, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

Eigen::MatrixXd TorqueCtrlModule::calcPoseError(Eigen::VectorXd target_position, Eigen::VectorXd current_position, Eigen::MatrixXd target_orientation, Eigen::MatrixXd current_orientation)
{
  Eigen::VectorXd position_error    =	target_position - current_position;
  Eigen::MatrixXd orientation_error =	current_orientation * robotis_framework::convertRotToOmega(current_orientation.inverse() * target_orientation);

  Eigen::VectorXd error = Eigen::VectorXd::Zero(TASK_DEMENSION);
  error(0) = position_error(0);
  error(1) = position_error(1);
  error(2) = position_error(2);
  error(3) = orientation_error.coeff(0,0);
  error(4) = orientation_error.coeff(1,0);
  error(5) = orientation_error.coeff(2,0);

  return error;
}

void TorqueCtrlModule::setCurrentPoseError()
{
  force_current_position_(0) = present_kinematics_pose_.position.x;
  force_current_position_(1) = present_kinematics_pose_.position.y;
  force_current_position_(2) = present_kinematics_pose_.position.z;

  Eigen::Quaterniond current_quaternion(present_kinematics_pose_.orientation.w,
                                        present_kinematics_pose_.orientation.x,
                                        present_kinematics_pose_.orientation.y,
                                        present_kinematics_pose_.orientation.z);
  force_current_orientation_ = robotis_framework::convertQuaternionToRotation(current_quaternion);
}

void TorqueCtrlModule::setTargetPoseError(int cnt)
{
  for (int dim=0; dim<3; dim++)
    force_target_position_(dim) = goal_task_tra_.coeff(cnt, dim);

  double step = (double) cnt / (double) all_time_steps_;
  Eigen::Quaterniond quaternion = initial_orientation_.slerp(step, target_orientation_);
  force_target_orientation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void TorqueCtrlModule::setKinematicsChain()
{
  if (using_gripper_ == true)
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

    if (using_gripper_ == true )
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
    if (using_gripper_ == true)
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
  }

  // Set Joint Limits
  std::vector<double> min_position_limit, max_position_limit;
  min_position_limit.push_back(-90.0);  max_position_limit.push_back(90.0); // joint1
  min_position_limit.push_back(-90.0);	max_position_limit.push_back(90.0); // joint2
  min_position_limit.push_back(-90.0);  max_position_limit.push_back(90.0); // joint3
  min_position_limit.push_back(-90.0);	max_position_limit.push_back(90.0); // joint4
  min_position_limit.push_back(-90.0);	max_position_limit.push_back(90.0); // joint5
  min_position_limit.push_back(-90.0);	max_position_limit.push_back(90.0); // joint6

  KDL::JntArray min_joint_position_limit(MAX_JOINT_NUM), max_joint_position_limit(MAX_JOINT_NUM);
  for (int index=0; index<MAX_JOINT_NUM; index++)
  {
    min_joint_position_limit(index) = min_position_limit[index]*DEGREE2RADIAN;
    max_joint_position_limit(index) = max_position_limit[index]*DEGREE2RADIAN;
  }

  /* KDL Solver Initialization */
  dyn_param_ = new KDL::ChainDynParam(chain_, KDL::Vector(0.0, 0.0, -9.81)); // kinematics & dynamics parameter
  jacobian_solver_ = new KDL::ChainJntToJacSolver(chain_); // jabocian solver
  forward_kinematics_solver_ = new KDL::ChainFkSolverPos_recursive(chain_); // forward kinematics solver

  // inverse kinematics solver
  inverse_vel_kinematics_solver_ = new KDL::ChainIkSolverVel_pinv(chain_);
  inverse_pos_kinematics_solver_ = new KDL::ChainIkSolverPos_NR_JL(chain_, min_joint_position_limit, max_joint_position_limit,
                                                                   *forward_kinematics_solver_,
                                                                   *inverse_vel_kinematics_solver_);
}

void TorqueCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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

    present_joint_position_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;
  }

  if (initialize_goal_value_==false)
  {
    goal_joint_position_ = present_joint_position_;
    goal_joint_velocity_ = present_joint_velocity_;
    goal_joint_effort_ = present_joint_effort_;

    initialize_goal_value_ = true;
  }

  /*----- Calculation -----*/
  calcGravityTerm();
  calcCoriolisTerm();
  calcMassTerm();

  calcForwardKinematics();
  calcJacobian();

  setCurrentPoseError();

  /*----- Set Goal Torque Data -----*/
  if (module_control_ == GRAVITY_COMPENSATION)
  {
    goal_joint_effort_ = gravity_term_;
  }
  else if (module_control_ == JOINT_CONTROL)
  {
    if (is_moving_ == true)
    {
      for (int index=0; index<MAX_JOINT_NUM; index++)
      {
        goal_joint_position_(index) = goal_joint_tra_(cnt_,index);
        goal_joint_velocity_(index) = goal_joint_tra_vel_(cnt_,index);
        goal_joint_acceleration_(index) = goal_joint_tra_accel_(cnt_,index);
      }
      cnt_++;
    }

    /* Computed Torque Method */
//    for (int it=0; it<MAX_JOINT_NUM; it++)
//    {
//      error_(it) = goal_joint_position_(it)-present_joint_position_(it);
//      derivative_(it) = goal_joint_velocity_(it) - present_joint_velocity_(it);

//      u_(it) =
//          goal_joint_acceleration_(it) +
//          d_gain_(it)*derivative_(it) +
//          p_gain_(it)*error_(it);
//    }
//    goal_joint_effort_ = mass_term_*u_ + coriolis_term_ + gravity_term_;

    /* PID Control with Gravity Compensation */
    for (int it=0; it<MAX_JOINT_NUM; it++)
    {
      error_(it) = goal_joint_position_(it)-present_joint_position_(it);
      integral_(it) = integral_(it) + (error_(it)*ITERATION_TIME);
      derivative_(it) = (error_(it)-error_prior_(it))/ITERATION_TIME;; // present_joint_velocity_(it);

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
    if (is_moving_ == true)
    {
      setTargetPoseError(cnt_);
      cnt_++;
    }

    goal_pose_error_ = calcPoseError(force_target_position_, force_current_position_,
                                     force_target_orientation_, force_current_orientation_);

    for (int index=0; index<MAX_JOINT_NUM; index++)
    {
      goal_task_wrench_(index) = force_p_gain_(index) * goal_pose_error_(index);
      goal_task_wrench_derivative_(index) = force_d_gain_(index) * ((goal_pose_error_(index) - goal_pose_error_prior_(index))/ITERATION_TIME);

      goal_joint_velocity_damping_(index) = 0.1 * present_joint_velocity_(index); //pose_joint_gain_(index) * present_joint_velocity_(index);
    }

    goal_joint_effort_ = jacobian_.transpose() * (goal_task_wrench_ + goal_task_wrench_derivative_) + gravity_term_;

    goal_pose_error_prior_ = goal_pose_error_;
  }

  sensor_msgs::JointState goal_joint_states_msg;

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_torque_ = goal_joint_effort_(joint_name_to_id_[joint_name]-1);

//    goal_joint_states_msg.name.push_back(joint_name);
//    goal_joint_states_msg.position.push_back(goal_joint_position_(joint_name_to_id_[joint_name]-1)); // goal position
//    goal_joint_states_msg.velocity.push_back(goal_joint_velocity_(joint_name_to_id_[joint_name]-1)); // goal velocity
//    goal_joint_states_msg.effort.push_back(goal_joint_acceleration_(joint_name_to_id_[joint_name]-1)); // goal acceleration
//    goal_joint_states_msg.header.stamp = ros::Time::now();
  }
//  goal_joint_states_pub_.publish(goal_joint_states_msg);


//  ros::Duration dur = ros::Time::now() - now;
//  double msec = dur.nsec * 0.000001;
//  ROS_INFO_STREAM("Process duration  : " << msec);

  if (module_control_ == JOINT_CONTROL || module_control_ == FORCE_CONTROL)
    closeMovementEvent();
}

void TorqueCtrlModule::stop()
{
  return;
}

bool TorqueCtrlModule::isRunning()
{
  return is_moving_;
}

void TorqueCtrlModule::closeMovementEvent()
{
  if (is_moving_ == true && cnt_ >= all_time_steps_)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

    is_moving_ = false;
    cnt_ = 0;
  }
}

void TorqueCtrlModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.type = type;
    status_msg.module_name = "TorqueCtrl";
    status_msg.status_msg = msg;

    status_msg_pub_.publish(status_msg);
}
