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
 *      Author: sch, Darby Lim
 */

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

  setKinematicsChain();

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ManipulatorX4PositionCtrlModule::queueThread, this));
}

void ManipulatorX4PositionCtrlModule::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue callback_queue;

  nh.setCallbackQueue(&callback_queue);

  status_msg_pub_  = nh.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 10);
  set_position_ctrl_module_msg_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/set_module_msg", 10,
                                   &ManipulatorX4PositionCtrlModule::setPositionCtrlModuleMsgCallback, this);

  joint_present_position_server_ = nh.advertiseService("/robotis/manipulator_x4/position_ctrl/joint_present_position",
                                                       &ManipulatorX4PositionCtrlModule::getJointPresentPositionMsgCallback, this);
  joint_kinematics_position_server_ = nh.advertiseService("/robotis/manipulator_x4/position_ctrl/joint_kinematics_position",
                                                       &ManipulatorX4PositionCtrlModule::getJointKinematicsPositionMsgCallback, this);

  enable_joint_control_mode_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/enable_joint_control_mode", 10,
                                                &ManipulatorX4PositionCtrlModule::enableJointSpaceControlModeMsgCallback, this);
  set_init_position_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/set_init_position", 10,
                                        &ManipulatorX4PositionCtrlModule::setInitPositionMsgCallback, this);
  set_zero_position_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/send_zero_position", 10,
                                        &ManipulatorX4PositionCtrlModule::setZeroPositionMsgCallback, this);
  joint_goal_position_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/send_goal_position", 10,
                                          &ManipulatorX4PositionCtrlModule::setJointGoalPositionMsgCallback, this);

  enable_task_space_control_mode_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/enable_tack_space_control_mode", 10,
                                                     &ManipulatorX4PositionCtrlModule::enableTaskSpaceControlModeMsgCallback, this);
  set_kinematics_pose_msg_sub_ = nh.subscribe("/robotis/manipulator_x4/position_ctrl/set_kinematics_position", 10,
                                              &ManipulatorX4PositionCtrlModule::setKinematicsPositionMsgCallback, this);

  while (nh.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void ManipulatorX4PositionCtrlModule::setPositionCtrlModuleMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Manipulator-X4 Poition Control Module");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Space Control Mode");

  jointSpaceControlMode_ = true;
  taskSpaceControlMode_ = false;
}

bool ManipulatorX4PositionCtrlModule::getJointPresentPositionMsgCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
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

bool ManipulatorX4PositionCtrlModule::getJointKinematicsPositionMsgCallback(manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Request &req,
                                                                            manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return false;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Kinematics Present Position");

  calcForwardKinematics();

  res.position_ctrl_kinematics_pose.group_name = "arm";
  res.position_ctrl_kinematics_pose.pose = present_kinematics_position_;

  return true;
}

void ManipulatorX4PositionCtrlModule::enableJointSpaceControlModeMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Space Control Mode");

  jointSpaceControlMode_ = true;
  taskSpaceControlMode_ = false;
}

void ManipulatorX4PositionCtrlModule::setInitPositionMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  if (is_moving_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Init Position");

    std::string pose_path;
    pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/initial_pose.yaml";

    parseIniPoseData(pose_path);
  }
  else
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
  }
}

void ManipulatorX4PositionCtrlModule::setZeroPositionMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  if (is_moving_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Zero Position");

    std::string pose_path;
    pose_path = ros::package::getPath("manipulator_x_position_ctrl_module") + "/config/zero_pose.yaml";

    parseIniPoseData(pose_path);
  }
  else
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
  }
}

void ManipulatorX4PositionCtrlModule::setJointGoalPositionMsgCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  if (jointSpaceControlMode_ == true)
  {
    if (is_moving_ == false)
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
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous Task is Alive");
    }
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

  jointSpaceControlMode_ = true;
  taskSpaceControlMode_ = false;

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

  jointSpaceControlMode_ = false;
  taskSpaceControlMode_ = true;
}

void ManipulatorX4PositionCtrlModule::setKinematicsChain(void)
{
//  if (using_gazebo_ == true)
//  {
    chain_.addSegment(KDL::Segment("Base",
                                   KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0.012, 0.0, 0.034)),
                                   KDL::RigidBodyInertia(0.08659,
                                                         KDL::Vector(0.00025, 0.0, -0.02420),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint1",
                                   KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame(KDL::Vector(0.0, 0.0, 0.04235)),
                                   KDL::RigidBodyInertia(0.117,
                                                         KDL::Vector(0.00002, -0.00045, 0.03293),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint2",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.024, 0.0, 0.12165)),
                                   KDL::RigidBodyInertia(0.156,
                                                         KDL::Vector(0.00966, -0.00034, 0.11795),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint3",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.1235, 0.0, 0.0)),
                                   KDL::RigidBodyInertia(0.149,
                                                         KDL::Vector(0.11076, -0.00035, 0.00050),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
    chain_.addSegment(KDL::Segment("Joint4",
                                   KDL::Joint(KDL::Joint::RotY),
                                   KDL::Frame(KDL::Vector(0.064, 0.0, 0.0)),
                                   KDL::RigidBodyInertia(0.124,
                                                         KDL::Vector(0.03767, 0.00262, -0.00043),
                                                         KDL::RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
                                                         )
                                   )
                      );
//  }
//  else
//  {

//  }

  // Set Joint Limits
  std::vector<double> min_position_limit, max_position_limit;
  min_position_limit.push_back(-160.0);   max_position_limit.push_back(160.0);  // joint1
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint2
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint3
  min_position_limit.push_back(-90.0);    max_position_limit.push_back(90.0);   // joint4

  KDL::JntArray min_joint_position_limit(MAX_JOINT_NUM), max_joint_position_limit(MAX_JOINT_NUM);
  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    min_joint_position_limit(index) = min_position_limit[index]*DEGREE2RADIAN;
    max_joint_position_limit(index) = max_position_limit[index]*DEGREE2RADIAN;
  }

  // Forward kinematics solver
  forward_kinematics_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);

  // Inverse kinematics solver
  inverse_pos_kinematics_solver_ = new KDL::ChainIkSolverPos_NR_JL(chain_, min_joint_position_limit, max_joint_position_limit,
                                                                   *forward_kinematics_solver_,
                                                                   *inverse_vel_kinematics_solver_);
}

void ManipulatorX4PositionCtrlModule::calcForwardKinematics()
{
  KDL::JntArray kdl_joint_position;
  kdl_joint_position.data = joint_goal_position_;

  KDL::Frame kdl_kinematics_position;
  forward_kinematics_solver_->JntToCart(kdl_joint_position, kdl_kinematics_position);

  present_kinematics_position_.position.x = kdl_kinematics_position.p.x();
  present_kinematics_position_.position.y = kdl_kinematics_position.p.y();
  present_kinematics_position_.position.z = kdl_kinematics_position.p.z();

  kdl_kinematics_position.M.GetQuaternion(present_kinematics_position_.orientation.x,
                                          present_kinematics_position_.orientation.y,
                                          present_kinematics_position_.orientation.z,
                                          present_kinematics_position_.orientation.w);
}

bool ManipulatorX4PositionCtrlModule::calcInverseKinematics(int cnt)
{
  calcForwardKinematics();

  double kdl_cnt = (double)cnt / (double)all_time_steps_;
  Eigen::Quaterniond kdl_orientation = initial_orientation_.slerp(kdl_cnt, target_orientation_);

  KDL::JntArray kdl_initial_joint_position;
  kdl_initial_joint_position.data = joint_goal_position_;

  KDL::Frame kdl_desired_kinematics_position;

  kdl_desired_kinematics_position.p.x(task_goal_trajectory_.coeff(cnt, 0));
  kdl_desired_kinematics_position.p.y(task_goal_trajectory_.coeff(cnt, 1));
  kdl_desired_kinematics_position.p.z(task_goal_trajectory_.coeff(cnt, 2));

  kdl_desired_kinematics_position.M = KDL::Rotation::Quaternion(kdl_orientation.x(),
                                                                kdl_orientation.y(),
                                                                kdl_orientation.z(),
                                                                kdl_orientation.w());

  KDL::JntArray kdl_desired_joint_position;
  kdl_desired_joint_position.resize(MAX_JOINT_NUM);

  if (inverse_pos_kinematics_solver_->CartToJnt(kdl_initial_joint_position, kdl_desired_kinematics_position, kdl_desired_joint_position) < 0)
  {
    is_moving_ = false;
    step_cnt_ = 0;

    return false;
  }

  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    joint_goal_position_(index) = kdl_desired_joint_position(index);
  }

  return true;
}

void ManipulatorX4PositionCtrlModule::calculateGoalTaskTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;
  task_goal_trajectory_.resize(all_time_steps_, 3);

  for (int index = 0; index < 3; index++)
  {
    double init_value = initial_position(index);
    double target_value = target_position(index);

    Eigen::MatrixXd trajectory =
        robotis_framework::calcMinimumJerkTra(init_value, 0.0, 0.0,
                                              target_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    task_goal_trajectory_.block(0, index, all_time_steps_, 1) = trajectory;
  }

  step_cnt_ = 0;
  is_moving_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void ManipulatorX4PositionCtrlModule::setKinematicsPositionMsgCallback(const manipulator_x_position_ctrl_module_msgs::KinematicsPose::ConstPtr &msg)
{
  if (enable_ == false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please, set position control module");
    return;
  }

  if (is_moving_ == false)
  {
    if (taskSpaceControlMode_ == true)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Set Joint Kinematics Position");

      Eigen::VectorXd initial_position = Eigen::VectorXd::Zero(3);
      Eigen::VectorXd target_position = Eigen::VectorXd::Zero(3);

      move_time_ = msg->move_time;

      calcForwardKinematics();

      initial_position(0) = present_kinematics_position_.position.x;
      initial_position(1) = present_kinematics_position_.position.y;
      initial_position(2) = present_kinematics_position_.position.z;

      target_position(0) = msg->pose.position.x;
      target_position(1) = msg->pose.position.y;
      target_position(2) = msg->pose.position.z;

      initial_orientation_.x() = present_kinematics_position_.orientation.x;
      initial_orientation_.y() = present_kinematics_position_.orientation.y;
      initial_orientation_.z() = present_kinematics_position_.orientation.z;
      initial_orientation_.w() = present_kinematics_position_.orientation.w;

      target_orientation_.x() = msg->pose.orientation.x;
      target_orientation_.y() = msg->pose.orientation.y;
      target_orientation_.z() = msg->pose.orientation.z;
      target_orientation_.w() = msg->pose.orientation.w;

      calculateGoalTaskTrajectory(initial_position, target_position);
    }
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

    joint_present_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_position_;
    joint_present_velocity_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_velocity_;
    joint_present_current_(joint_id_[joint_name]-1) = dxl->dxl_state_->present_torque_;

    joint_goal_position_(joint_id_[joint_name]-1) = dxl->dxl_state_->goal_position_;
  }

  /* Set Joint Pose */
  if (is_moving_ == true)
  {
    if (jointSpaceControlMode_ == true)
    {
      for (int index = 0; index < MAX_JOINT_NUM; index++)
      {
        joint_goal_position_(index) = joint_goal_trajectory_(step_cnt_, index);
      }
    }
    else if (taskSpaceControlMode_ == true)
    {
      bool ik_success = calcInverseKinematics(step_cnt_);

      if (ik_success == false)
      {
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Failed to Solve Inverse Kinematics");
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");
      }
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
  return is_moving_;
}

void ManipulatorX4PositionCtrlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Manipulator_X4_Position_Ctrl";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
