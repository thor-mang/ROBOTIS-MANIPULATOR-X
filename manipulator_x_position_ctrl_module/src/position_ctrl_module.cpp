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
    using_gazebo_(),
    is_moving_(false),
    move_time_(0.0),
    all_time_steps_(0),
    step_cnt_(0)
{
  enable_ = false; //motion_module.h
  module_name_ = "manipulator_x4_position_ctrl";
  control_mode_ = robotis_framework::PositionControl;

  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();

  joint_id_["joint1"] = 1;
  joint_id_["joint2"] = 2;
  joint_id_["joint3"] = 3;
  joint_id_["joint4"] = 4;

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
  set_module_msg_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/set_module_msg", 10,
                                   &ManipulatorX4PositionCtrlModule::setModuleMsgCallback, this);

  enable_joint_control_mode_sub_ = nh.subscribe("robotis/manipulator_x4_position_ctrl/enable_joint_control_mode", 10,
                                                &ManipulatorX4PositionCtrlModule::enableJointControlModeMsgCallback, this);
  set_init_position_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/set_init_position", 10,
                                        &ManipulatorX4PositionCtrlModule::setInitPoseMsgCallback, this);
  set_zero_position_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/send_zero_position", 10,
                                        &ManipulatorX4PositionCtrlModule::setZeroPoseMsgCallback, this);
  joint_present_position_server_ = nh.advertiseService("/robotis/manipulator_x4_position_ctrl/joint_present_position",
                                                       &ManipulatorX4PositionCtrlModule::getJointPresentPositionCallback, this);
  joint_goal_position_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/send_goal_position", 10,
                                          &ManipulatorX4PositionCtrlModule::setJointGoalPositionCallback, this);

  enable_task_space_control_mode_sub_ = nh.subscribe("/robotis/manipulator_x4_position_ctrl/enable_tack_space_control_mode", 10,
                                                     &ManipulatorX4PositionCtrlModule::enableTaskSpaceControlModeMsgCallback, this);

  while (nh.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void ManipulatorX4PositionCtrlModule::setModuleMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Manipulator-X4 Poition Control Module");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Control Mode");

  jointControlMode_ = true;
  taskspaceControlMode_ = false;

//  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Init Position");

//  std::string pose_path;
//  pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/initial_pose.yaml";

//  parseIniPoseData(pose_path);
}

void ManipulatorX4PositionCtrlModule::enableJointControlModeMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Control Mode");

  jointControlMode_ = true;
  taskspaceControlMode_ = false;
}

void ManipulatorX4PositionCtrlModule::setInitPoseMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Init Position");

  std::string pose_path;
  pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/initial_pose.yaml";

  parseIniPoseData(pose_path);
}

void ManipulatorX4PositionCtrlModule::setZeroPoseMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Zero Position");

  std::string pose_path;
  pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/zero_pose.yaml";

  parseIniPoseData(pose_path);
}

bool ManipulatorX4PositionCtrlModule::getJointPresentPositionCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
                                                                      manipulator_x_position_ctrl_module_msgs::GetJointPose::Response &res)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return false;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Joint Present Position");

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    res.position_ctrl_joint_pose.joint_name.push_back(joint_name);
    res.position_ctrl_joint_pose.position.push_back(joint_present_position_(joint_id_[joint_name]-1));
  }

  return true;
}

void ManipulatorX4PositionCtrlModule::setJointGoalPositionCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  if (jointControlMode_ == true)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Goal Position");

    Eigen::VectorXd initial_position = joint_goal_position_;
    Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

    move_time_ = msg->move_time;

    for (int it = 0; it < msg->joint_name.size(); it++)
    {
      target_position(joint_id_[msg->joint_name[it]]-1) = msg->position[it];
    }

    calculateGoalJointTrajectory(initial_position, target_position);
  }
  else
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Please, set Joint control mode");
  }
}

void ManipulatorX4PositionCtrlModule::calculateGoalJointTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  /* set movement time */
  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  joint_goal_trajectory_.resize(all_time_steps_, MAX_JOINT_NUM);

  /* calculate joint trajectory */
  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    double init_position_value = initial_position(index);
    double target_position_value = target_position(index);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(init_position_value, 0.0, 0.0,
                                              target_position_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    // Block of size (p,q), starting at (i,j)
    // block(i,j,p,q)
    joint_goal_trajectory_.block(0, index, all_time_steps_, 1) = tra;
  }

  step_cnt_ = 0;
  is_moving_ = true;
}

void ManipulatorX4PositionCtrlModule::parseIniPoseData(const std::__cxx11::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  jointControlMode_ = true;
  taskspaceControlMode_ = false;

  Eigen::VectorXd initial_position = joint_goal_position_;

  // parse movement time
  move_time_ = doc["move_time"].as<double>();

  // parse target position
  Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  YAML::Node target_position_node = doc["target_position"];
  for(YAML::iterator it = target_position_node.begin(); it != target_position_node.end(); ++it)
  {
    std::string joint_name;
    double value;

    joint_name = it->first.as<std::string>();
    value = it->second.as<double>();

    target_position(joint_id_[joint_name]-1) = value * DEGREE2RADIAN;
//    ROS_INFO("target_position [%2.2f]", target_position(it))
  }

  calculateGoalJointTrajectory(initial_position, target_position);
}

void ManipulatorX4PositionCtrlModule::enableTaskSpaceControlModeMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Task Space Control Mode");

  jointControlMode_ = false;
  taskspaceControlMode_ = true;
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

    joint_present_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    joint_present_velocity_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    joint_present_current_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;

    joint_goal_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->goal_position_;
  }

  /* Set Joint Pose */
  if (is_moving_ == true)
  {
    if (jointControlMode_ == true)
    {
      for (int index = 0; index < MAX_JOINT_NUM; index++)
      {
        joint_goal_position_(index) = joint_goal_trajectory_(step_cnt_, index);
      }
    }
    else if (taskspaceControlMode_ == true)
    {
        //TODO
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
