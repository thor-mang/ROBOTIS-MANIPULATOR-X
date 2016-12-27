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
 * position_ctrl_module.h
 *
 *  Created on: Jul 6, 2016
 *      Author: sch, Darby Lim
 */

#ifndef MOTION_MODULE_MANIPULATOR_X4_MOTION_MODULE_H
#define MOTION_MODULE_MANIPULATOR_X4_MOTION_MODULE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include <fstream>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "manipulator_x_position_ctrl_module_msgs/JointPose.h"
#include "manipulator_x_position_ctrl_module_msgs/GetJointPose.h"
#include "manipulator_x_position_ctrl_module_msgs/KinematicsPose.h"
#include "manipulator_x_position_ctrl_module_msgs/GetKinematicsPose.h"

namespace manipulator_x4_position_ctrl_module
{
#define MAX_JOINT_NUM  (4)
#define ITERATION_TIME (0.008)

class ManipulatorX4PositionCtrlModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<ManipulatorX4PositionCtrlModule>
{
 private:
  bool using_gazebo_;

  int control_cycle_msec_;
  boost::thread queue_thread_;

  // Joint states
  std::map<std::string, uint8_t> joint_id_;

  Eigen::VectorXd joint_present_position_;
  Eigen::VectorXd joint_present_velocity_;
  Eigen::VectorXd joint_present_current_;
  Eigen::VectorXd joint_goal_position_;

  // Publisher
  ros::Publisher status_msg_pub_;
  ros::ServiceServer joint_present_position_server_;
  ros::ServiceServer joint_kinematics_position_server_;

  // Subscriber
  ros::Subscriber set_position_ctrl_module_msg_sub_;

  ros::Subscriber enable_joint_control_mode_sub_;
  ros::Subscriber set_init_position_sub_;
  ros::Subscriber set_zero_position_sub_;
  ros::Subscriber joint_goal_position_sub_;

  ros::Subscriber enable_task_space_control_mode_sub_;
  ros::Subscriber set_kinematics_pose_msg_sub_;

  // Control Mode
  bool jointSpaceControlMode_;
  bool taskSpaceControlMode_;

  // Trajectory
  bool is_moving_;
  double move_time_;
  int all_time_steps_;
  int step_cnt_;

  // Joint Space Control
  Eigen::MatrixXd joint_goal_trajectory_;

  // Task Space Control
  KDL::Chain chain_;
  KDL::ChainFkSolverPos_recursive *forward_kinematics_solver_;
  KDL::ChainIkSolverVel_pinv *inverse_vel_kinematics_solver_;
  KDL::ChainIkSolverPos_NR_JL *inverse_pos_kinematics_solver_;

  geometry_msgs::Pose present_kinematics_position_;

  Eigen::MatrixXd task_goal_trajectory_;
  Eigen::Quaterniond initial_orientation_, target_orientation_;

  void queueThread();
  void publishStatusMsg(unsigned int type, std::string msg);
  void setPositionCtrlModuleMsgCallback(const std_msgs::String::ConstPtr &msg);

  bool getJointPresentPositionMsgCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
                                       manipulator_x_position_ctrl_module_msgs::GetJointPose::Response &res);
  bool getJointKinematicsPositionMsgCallback(manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Request &req,
                                             manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Response &res);

  void enableJointSpaceControlModeMsgCallback(const std_msgs::String::ConstPtr &msg);
  void setInitPositionMsgCallback(const std_msgs::String::ConstPtr &msg);
  void setZeroPositionMsgCallback(const std_msgs::String::ConstPtr &msg);
  void setJointGoalPositionMsgCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr &msg);

  void enableTaskSpaceControlModeMsgCallback(const std_msgs::String::ConstPtr &msg);
  void setKinematicsPositionMsgCallback(const manipulator_x_position_ctrl_module_msgs::KinematicsPose::ConstPtr &msg);

  void calculateGoalJointTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
  void calculateGoalTaskTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
  void parseIniPoseData(const std::string &path);
  void setKinematicsChain(void);
  void calcJacobian();
  void calcForwardKinematics();
  bool calcInverseKinematics(int cnt);

 public:
  ManipulatorX4PositionCtrlModule();
  virtual ~ManipulatorX4PositionCtrlModule();

  void initialize(const int control_cycle_msec_, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};
}

#endif /*MOTION_MODULE_MANIPULATOR_X4_MOTION_MODULE_H*/
