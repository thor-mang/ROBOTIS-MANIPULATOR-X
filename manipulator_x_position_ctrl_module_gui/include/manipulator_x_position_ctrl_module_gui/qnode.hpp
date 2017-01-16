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
 * qnode.hpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch, Darby Lim
 */

#ifndef MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_QNODE_HPP_
#define MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "manipulator_x_position_ctrl_module_msgs/JointPose.h"
#include "manipulator_x_position_ctrl_module_msgs/GetJointPose.h"
#include "manipulator_x_position_ctrl_module_msgs/KinematicsPose.h"
#include "manipulator_x_position_ctrl_module_msgs/GetKinematicsPose.h"
#endif

namespace manipulator_x_position_ctrl_module_gui
{
class QNode : public QThread
{
Q_OBJECT
 public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
  void run();

  enum LogLevel{
   Debug,
   Info,
   Warn,
   Error,
   Fatal};

  QStringListModel* loggingModel() {return &logging_model;}
  void log(const LogLevel &level, const std::string &msg, std::string sender);

  void sendSetModuleMsg(std_msgs::String msg);
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

  void setZeroPosition(std_msgs::String msg);
  void setInitPosition(std_msgs::String msg);

  void sendGripperGoalPositionMsg(std_msgs::Float64 msg);

  void sendEnableJointControlMode(std_msgs::String msg);
  void sendJointGoalPositionMsg(manipulator_x_position_ctrl_module_msgs::JointPose msg);
  void getJointPresentPosition(void);

  void sendEnableTaskSpaceControlMode(std_msgs::String msg);
  void sendKinematicsPositionMsg(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg);
  void getKinematicsPresentPosition(void);

  void sendEnableMotionPlanningMode(std_msgs::String msg);
  void sendMotionPlanningTargetPoseMsg(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();

  void updateJointPresentPosition(manipulator_x_position_ctrl_module_msgs::JointPose msg);
  void updateKinematicsPresentPose(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg);

private:
	int init_argc;
	char** init_argv;

  ros::Subscriber status_msg_sub_;
  ros::ServiceClient joint_present_position_client_;

  ros::Publisher set_position_ctrl_module_msg_pub_;

  ros::Publisher set_gripper_module_msg_pub_;
  ros::Publisher gripper_goal_position_pub_;

  ros::Publisher enable_joint_control_mode_pub_;
  ros::Publisher set_init_position_pub_;
  ros::Publisher set_zero_position_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher send_goal_joint_position_pub_;

  ros::Publisher enable_task_space_control_mode_pub_;
  ros::Publisher set_kinematics_pose_msg_pub_;
  ros::ServiceClient kinematics_present_pose_client_;

  ros::Publisher enable_motion_planning_mode_pub_;
  ros::Publisher set_motion_planning_pose_msg_pub_;

  QStringListModel logging_model;
};
}  // namespace manipulator_x_position_ctrl_module_gui

#endif /* MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_QNODE_HPP_ */
