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
 *      Author: sch
 */

#ifndef SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_POSITION_CTRL_MODULE_INCLUDE_MANIPULATOR_X_POSITION_CTRL_MODULE_POSITION_CTRL_MODULE_H_
#define SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_POSITION_CTRL_MODULE_INCLUDE_MANIPULATOR_X_POSITION_CTRL_MODULE_POSITION_CTRL_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>

//#include <kdl/joint.hpp>
//#include <kdl/chain.hpp>
//#include <kdl/chaindynparam.hpp>
//#include <kdl/jacobian.hpp>
//#include <kdl/chainjnttojacsolver.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_nr_jl.hpp>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "manipulator_x_position_ctrl_module_msgs/JointPose.h"
//#include "manipulator_x_position_ctrl_module_msgs/KinematicsPose.h"

#include "manipulator_x_position_ctrl_module_msgs/GetJointPose.h"
//#include "manipulator_x_position_ctrl_module_msgs/GetKinematicsPose.h"

namespace robotis_manipulator_x
{

//#define MAX_JOINT_NUM   (7)
#define MAX_JOINT_NUM   (6)
#define TASK_DEMENSION  (6)

enum MODE_SELECT
{
  NONE,
  JOINT_SPACE_CONTROL,
  TASK_SPACE_CONTROL
};

class PositionCtrlModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<PositionCtrlModule>
{
private:
  double control_cycle_sec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Publisher  status_msg_pub_;
  ros::Publisher  set_ctrl_module_pub_;

  ros::ServiceServer get_joint_pose_server_;
  ros::ServiceServer get_kinematics_pose_server_;

  std::map<std::string, int> joint_name_to_id_;

  bool using_gazebo_;

  MODE_SELECT module_control_;

//  KDL::Chain chain_;
//  KDL::ChainDynParam *dyn_param_ = NULL;
//  KDL::ChainJntToJacSolver *jacobian_solver_;
//  KDL::ChainFkSolverPos_recursive *forward_kinematics_solver_;
//  KDL::ChainIkSolverVel_pinv *inverse_vel_kinematics_solver_;
//  KDL::ChainIkSolverPos_NR_JL *inverse_pos_kinematics_solver_;

//  Eigen::MatrixXd jacobian_;
//  geometry_msgs::Pose present_kinematics_pose_;

  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd present_joint_effort_;
  Eigen::VectorXd goal_joint_position_;

  /* Definition for Trajectory */
  bool is_moving_;
  double mov_time_;
  int all_time_steps_, cnt_;

  /* Definition for JOINT SPACE CONTROL */
  Eigen::MatrixXd goal_joint_tra_;

  /* Definition for TASK SPACE CONTROL */
//  Eigen::MatrixXd goal_task_tra_;
//  Eigen::Quaterniond initial_orientation_, target_orientation_;

  void queueThread();

//  void setKinematicsChain();
//  void calcJacobian();
//  void calcForwardKinematics();
//  bool calcInverseKinematics(int cnt);
  void calcGoalJointTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
//  void calcGoalTaskTra(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);

public:
  PositionCtrlModule();
  virtual ~PositionCtrlModule();

  /* ROS Topic Callback Functions */
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setInitialPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void parseIniPoseData( const std::string &path );

  void enableJointSpaceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg);
//  void enableTaskSpaceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg);

  void setJointPoseMsgCallback(const manipulator_x_position_ctrl_module_msgs::JointPose::ConstPtr& msg);
//  void setKinematicsPoseMsgCallback(const manipulator_x_position_ctrl_module_msgs::KinematicsPose::ConstPtr& msg);

  bool getJointPoseCallback(manipulator_x_position_ctrl_module_msgs::GetJointPose::Request &req,
                            manipulator_x_position_ctrl_module_msgs::GetJointPose::Response &res);
//  bool getKinematicsPoseCallback(manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Request &req,
//                                 manipulator_x_position_ctrl_module_msgs::GetKinematicsPose::Response &res);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);
};

}

#endif /* SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_POSITION_CTRL_MODULE_INCLUDE_MANIPULATOR_X_POSITION_CTRL_MODULE_POSITION_CTRL_MODULE_H_ */
