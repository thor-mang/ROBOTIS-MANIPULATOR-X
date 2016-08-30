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
  : control_cycle_sec_(0.008),
    using_gazebo_(),
    is_moving_(false),
    module_control_(NONE)
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
  result_["joint7"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;
  joint_name_to_id_["joint5"] = 5;
  joint_name_to_id_["joint6"] = 6;
  joint_name_to_id_["joint7"] = 7;

  present_joint_position_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  present_joint_velocity_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  present_joint_effort_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_joint_position_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
}

PositionCtrlModule::~PositionCtrlModule()
{
  queue_thread_.join();
}

void PositionCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle ros_node;
  ros_node.getParam("gazebo", using_gazebo_);

  setKinematicsChain();

  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&PositionCtrlModule::queueThread, this));
}

void PositionCtrlModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  get_joint_pose_server_ = ros_node.advertiseService("/robotis/position_ctrl/get_joint_pose",
                                                     &PositionCtrlModule::getJointPoseCallback, this);
  get_kinematics_pose_server_ = ros_node.advertiseService("/robotis/position_ctrl/get_kinematics_pose",
                                                          &PositionCtrlModule::getKinematicsPoseCallback, this);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/position_ctrl/set_mode_msg", 5,
                                                        &PositionCtrlModule::setModeMsgCallback, this);
  ros::Subscriber set_initial_pose_msg_sub = ros_node.subscribe("/robotis/position_ctrl/set_initial_pose_msg", 5,
                                                                &PositionCtrlModule::setInitialPoseMsgCallback, this);
  ros::Subscriber enable_joint_space_control_msg_sub = ros_node.subscribe("/robotis/position_ctrl/enable_joint_space_control_msg", 5,
                                                                          &PositionCtrlModule::enableJointSpaceControlMsgCallback, this);
  ros::Subscriber enable_task_space_control_msg_sub = ros_node.subscribe("/robotis/position_ctrl/enable_task_space_control_msg", 5,
                                                                         &PositionCtrlModule::enableTaskSpaceControlMsgCallback, this);

  ros::Subscriber set_joint_pose_msg_sub = ros_node.subscribe("/robotis/position_ctrl/set_joint_pose_msg", 5,
                                                              &PositionCtrlModule::setJointPoseMsgCallback, this);
  ros::Subscriber set_kinematics_pose_msg_sub = ros_node.subscribe("/robotis/position_ctrl/set_kinematics_pose_msg", 5,
                                                                   &PositionCtrlModule::setKinematicsPoseMsgCallback, this);

  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void PositionCtrlModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Poition Control Module");
  std_msgs::String str_msg;
  str_msg.data = "position_ctrl_module";
  set_ctrl_module_pub_.publish(str_msg);
}

void PositionCtrlModule::setInitialPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if (is_moving_ == false)
  {
    std::string ini_pose_path;

    if (msg->data == "zero_pose")
      ini_pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/zero_pose.yaml";
    else if (msg->data == "initial_pose")
      ini_pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/initial_pose.yaml";

    parseIniPoseData(ini_pose_path);
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
}

void PositionCtrlModule::parseIniPoseData(const std::string &path)
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

  module_control_ = JOINT_SPACE_CONTROL;
  Eigen::VectorXd initial_position = goal_joint_position_;

  // parse movement time
  mov_time_ = doc["mov_time"].as< double >();

  // parse target pose
  Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  YAML::Node tar_pose_node = doc["tar_pose"];
  for(YAML::iterator it = tar_pose_node.begin() ; it != tar_pose_node.end() ; ++it)
  {
    std::string joint_name;
    double value;

    joint_name = it->first.as<std::string>();
    value = it->second.as<double>();

    target_position(joint_name_to_id_[joint_name]-1) = value * DEGREE2RADIAN;
  }

  calcGoalJointTra(initial_position, target_position);
}

void PositionCtrlModule::enableJointSpaceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if (msg->data == true)
  {
    module_control_ = JOINT_SPACE_CONTROL;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Enable Joint Space Control");
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Disable Joint Space Control");
}

void PositionCtrlModule::enableTaskSpaceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if (msg->data == true)
  {
    module_control_ = TASK_SPACE_CONTROL;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Enable Task Space Control");
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Disable Task Space Control");
}

void PositionCtrlModule::setJointPoseMsgCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if(is_moving_ == false)
  {
    if (module_control_ == JOINT_SPACE_CONTROL)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Goal Joint Values");

      Eigen::VectorXd initial_position = goal_joint_position_;
      Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

      mov_time_ = msg->mov_time;

      for (int it=0; it<msg->joint_name.size(); it++)
        target_position(joint_name_to_id_[msg->joint_name[it]]-1) = msg->position[it];

      calcGoalJointTra(initial_position, target_position);
    }
    else
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Check Enable Joint Space Control");
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
}

void PositionCtrlModule::setKinematicsPoseMsgCallback(const manipulator_x_position_ctrl_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  if (is_moving_ == false)
  {
    if (module_control_ == TASK_SPACE_CONTROL)
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
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Check Enable Task Space Control");
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
}

bool PositionCtrlModule::getJointPoseCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
                                              manipulator_x_position_ctrl_module_msgs::GetJointPose::Response &res)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return false;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Present Joint Values");

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.position_ctrl_joint_pose.joint_name.push_back(it->first);
    res.position_ctrl_joint_pose.position.push_back(goal_joint_position_(joint_name_to_id_[it->first]-1));
  }

  calcJacobian();

  return true;
}

bool PositionCtrlModule::getKinematicsPoseCallback(manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Request &req,
                                                   manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return false;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Present Kinematics Pose");

  calcForwardKinematics();

  res.position_ctrl_kinematics_pose.group_name = "arm";
  res.position_ctrl_kinematics_pose.pose = present_kinematics_pose_;

  return true;
}

void PositionCtrlModule::calcJacobian()
{
  KDL::JntArray kdl_joint_position;
  kdl_joint_position.data = goal_joint_position_;

  KDL::Jacobian jacobian(MAX_JOINT_NUM);
  jacobian_solver_->JntToJac(kdl_joint_position,jacobian);

  jacobian_ = jacobian.data;
}

void PositionCtrlModule::calcForwardKinematics()
{
  KDL::JntArray kdl_joint_position;
  kdl_joint_position.data = goal_joint_position_;

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

bool PositionCtrlModule::calcInverseKinematics(int cnt)
{
  calcForwardKinematics();

  double kdl_cnt = (double) cnt / (double) all_time_steps_;
  Eigen::Quaterniond kdl_orientation = initial_orientation_.slerp(kdl_cnt, target_orientation_);

  KDL::JntArray kdl_initial_joint_position;
  kdl_initial_joint_position.data = goal_joint_position_;

  KDL::Frame kdl_desired_kinematics_pose;

  kdl_desired_kinematics_pose.p.x(goal_task_tra_.coeff(cnt,0));
  kdl_desired_kinematics_pose.p.y(goal_task_tra_.coeff(cnt,1));
  kdl_desired_kinematics_pose.p.z(goal_task_tra_.coeff(cnt,2));

  kdl_desired_kinematics_pose.M = KDL::Rotation::Quaternion(kdl_orientation.x(),
                                                            kdl_orientation.y(),
                                                            kdl_orientation.z(),
                                                            kdl_orientation.w());

  KDL::JntArray kdl_desired_joint_position;
  kdl_desired_joint_position.resize(MAX_JOINT_NUM);

  if (inverse_pos_kinematics_solver_->CartToJnt(kdl_initial_joint_position, kdl_desired_kinematics_pose, kdl_desired_joint_position) < 0)
  {
    is_moving_ = false;
    cnt_ = 0;

    return false;
  }

  for (int index=0; index<MAX_JOINT_NUM; index++)
    goal_joint_position_(index) = kdl_desired_joint_position(index);

  return true;
}

void PositionCtrlModule::calcGoalJointTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  /* set movement time */
  all_time_steps_ = int(floor((mov_time_ / control_cycle_sec_ ) + 1.0));
  mov_time_ = double(all_time_steps_ - 1) * control_cycle_sec_;
  goal_joint_tra_.resize(all_time_steps_ , MAX_JOINT_NUM);

  /* calculate joint trajectory */
  for (int index=0; index<MAX_JOINT_NUM; index++)
  {
    double ini_value = initial_position(index);
    double tar_value = target_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value, 0.0, 0.0,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, index, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void PositionCtrlModule::calcGoalTaskTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
    return;
  }

  /* set movement time */
  all_time_steps_ = int(floor((mov_time_ / control_cycle_sec_) + 1.0));
  mov_time_ = double(all_time_steps_ - 1) * control_cycle_sec_;
  goal_task_tra_.resize(all_time_steps_, 3);

  /* calculate task position trajectory */
  for (int index=0; index<3; index++)
  {
    double ini_value = initial_position(index);
    double tar_value = target_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value, 0.0, 0.0,
                                              control_cycle_sec_, mov_time_);

    goal_task_tra_.block(0, index, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void PositionCtrlModule::setKinematicsChain()
{
  if (using_gazebo_ == true)
  {
    chain_.addSegment(KDL::Segment("Base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.0, 0.0, 0.042)),
                                   KDL::RigidBodyInertia(0.08659,
                                                         KDL::Vector(0.00025, 0.0, -0.02420),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, -0.019, 0.028)),
                                   KDL::RigidBodyInertia(0.00795,
                                                         KDL::Vector(0.0, 0.019, -0.02025),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.0, 0.019, 0.0405)),
                                   KDL::RigidBodyInertia(0.09312,
                                                         KDL::Vector(0.00000, -0.00057, -0.02731),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint3",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.024, -0.019, 0.064)),
                                   KDL::RigidBodyInertia(0.19398,
                                                         KDL::Vector(-0.02376, 0.01864, -0.02267),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint4",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.064, 0.019, 0.024)),
                                   KDL::RigidBodyInertia(0.09824,
                                                         KDL::Vector(-0.02099, 0.0, -0.01213),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint5",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.0405, -0.019, 0.0)),
                                   KDL::RigidBodyInertia(0.09312,
                                                         KDL::Vector(-0.01319, 0.01843, 0.0),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint6",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.064, 0.019, 0.0)),
                                   KDL::RigidBodyInertia(0.09824,
                                                         KDL::Vector(-0.02099, 0.0, 0.01142),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint7",
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
    chain_.addSegment(KDL::Segment("Base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.0, 0.0, 0.042)),
                                   KDL::RigidBodyInertia(0.08659,
                                                         KDL::Vector(0.00025, 0.0, -0.02420),
                                                         KDL::RotationalInertia(0.00001371, 0.00002369, 0.00002097, 0.0, -0.00000025, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, -0.019, 0.028)),
                                   KDL::RigidBodyInertia(0.00795,
                                                         KDL::Vector(0.0, 0.019, -0.02025),
                                                         KDL::RotationalInertia(0.00000265, 0.00000105, 0.00000246, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.0, 0.019, 0.0405)),
                                   KDL::RigidBodyInertia(0.09312,
                                                         KDL::Vector(0.00000, -0.00057, -0.02731),
                                                         KDL::RotationalInertia(0.00002889, 0.00002519, 0.00001549, 0.0, -0.00000012, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint3",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.024, -0.019, 0.064)),
                                   KDL::RigidBodyInertia(0.19398,
                                                         KDL::Vector(-0.02376, 0.01864, -0.02267),
                                                         KDL::RotationalInertia(0.00012132, 0.00016807, 0.00007952, 0.00000108, -0.000048, 0.00000157)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint4",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.064, 0.019, 0.024)),
                                   KDL::RigidBodyInertia(0.09824,
                                                         KDL::Vector(-0.02099, 0.0, -0.01213),
                                                         KDL::RotationalInertia(0.00002597, 0.00003222, 0.00002303, 0.0, -0.00000146, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint5",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.0405, -0.019, 0.0)),
                                   KDL::RigidBodyInertia(0.09312,
                                                         KDL::Vector(-0.01319, 0.01843, 0.0),
                                                         KDL::RotationalInertia(0.00001549, 0.00002519, 0.00002889, 0.00000012, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint6",
                                   KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   KDL::Frame(KDL::Vector(0.064, 0.019, 0.0)),
                                   KDL::RigidBodyInertia(0.09824,
                                                         KDL::Vector(-0.02099, 0.0, 0.01142),
                                                         KDL::RotationalInertia(0.00002594, 0.00003219, 0.00002303, 0.0, -0.00000085, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint7",
                                   KDL::Joint(KDL::Joint::RotX),
                                   KDL::Frame(KDL::Vector(0.14103, 0.0, 0.0)),
                                   KDL::RigidBodyInertia(0.26121,
                                                         KDL::Vector(-0.09906, 0.00146, -0.00021),
                                                         KDL::RotationalInertia(0.00019, 0.00022, 0.00029, 0.00001, 0.0, 0.0)
                                                         )
                                   )
                      );
  }

  // Set Joint Limits
  std::vector<double> min_position_limit, max_position_limit;
  min_position_limit.push_back(-160.0);   max_position_limit.push_back(160.0);  // joint1
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint2
  min_position_limit.push_back(-160.0);   max_position_limit.push_back(160.0);  // joint3
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint4
  min_position_limit.push_back(-160.0);   max_position_limit.push_back(160.0);  // joint5
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint6
  min_position_limit.push_back(-160.0);   max_position_limit.push_back(160.0);  // joint7

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

void PositionCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                 std::map<std::string, double> sensors)
{
  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
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
    if (module_control_==JOINT_SPACE_CONTROL)
    {
      for (int index=0; index<MAX_JOINT_NUM; index++)
        goal_joint_position_(index) = goal_joint_tra_(cnt_,index);
    }
    else if (module_control_==TASK_SPACE_CONTROL)
    {
      bool ik_success = calcInverseKinematics(cnt_);

      if (ik_success==false)
      {
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Failed to Solve Inverse Kinematics");
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");
      }
    }

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

void PositionCtrlModule::stop()
{
  return;
}

bool PositionCtrlModule::isRunning()
{
  return is_moving_;
}

void PositionCtrlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "PositionCtrl";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
