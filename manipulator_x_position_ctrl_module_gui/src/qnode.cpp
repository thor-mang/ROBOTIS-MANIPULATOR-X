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
 * qnode.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch, Darby Lim
 */

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/manipulator_x_position_ctrl_module_gui/qnode.hpp"

namespace manipulator_x_position_ctrl_module_gui
{
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
	wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"manipulator_x_position_ctrl_module_gui");
  if ( ! ros::master::check() )
  {
		return false;
	}
  ros::start();
  ros::NodeHandle nh;

	// Add your ros communications here.
  status_msg_sub_ = nh.subscribe("/robotis/status", 10, &QNode::statusMsgCallback, this);

  set_ctrl_module_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 10);
  set_position_ctrl_module_msg_pub_ = nh.advertise<std_msgs::String>("/robotis/manipulator_x4/position_ctrl/set_module_msg", 10);
  set_gripper_module_msg_pub_ = nh.advertise<std_msgs::String>("/robotis/manipulator_x4/gripper/set_module_msg", 10);

  gripper_goal_position_pub_ = nh.advertise<std_msgs::Float64>("/robotis/manipulator_x4/gripper/send_goal_position", 10);

  enable_joint_control_mode_pub_ = nh.advertise<std_msgs::String>(
                                  "/robotis/manipulator_x4/position_ctrl/enable_joint_control_mode", 10);
  set_init_position_pub_ = nh.advertise<std_msgs::String>(
                                  "/robotis/manipulator_x4/position_ctrl/set_init_position", 10);
  set_zero_position_pub_ = nh.advertise<std_msgs::String>(
                                  "/robotis/manipulator_x4/position_ctrl/set_zero_position", 10);
  send_goal_joint_position_pub_ = nh.advertise<manipulator_x_position_ctrl_module_msgs::JointPose>(
                                  "/robotis/manipulator_x4/position_ctrl/send_goal_position", 10);
  joint_present_position_client_ = nh.serviceClient<manipulator_x_position_ctrl_module_msgs::GetJointPose>(
                                  "/robotis/manipulator_x4/position_ctrl/joint_present_position", 10);
  enable_task_space_control_mode_pub_ = nh.advertise<std_msgs::String>(
                                  "/robotis/manipulator_x4/position_ctrl/enable_tack_space_control_mode", 10);
  set_kinematics_pose_msg_pub_ = nh.advertise<manipulator_x_position_ctrl_module_msgs::KinematicsPose>(
                                 "/robotis/manipulator_x4/position_ctrl/kinematics_target_pose", 10);
  kinematics_present_pose_client_ = nh.serviceClient<manipulator_x_position_ctrl_module_msgs::GetKinematicsPose>(
                                  "/robotis/manipulator_x4/position_ctrl/kinematics_present_pose", 10);

  enable_motion_planning_mode_pub_ = nh.advertise<std_msgs::String>(
                                  "/robotis/manipulator_x4/position_ctrl/enable_motion_planning_mode", 10);
  set_motion_planning_pose_msg_pub_ = nh.advertise<manipulator_x_position_ctrl_module_msgs::KinematicsPose>(
                                 "/robotis/manipulator_x4/position_ctrl/motion_planning_target_pose", 10);

  getJointPresentPosition();

  start();
	return true;
}

void QNode::sendSetModuleMsg(std_msgs::String msg)
{
  std_msgs::String str_msg;

  str_msg.data = "manipulator_x4_position_ctrl";
  set_ctrl_module_pub_.publish(str_msg);

  str_msg.data = "gripper_module";
  set_ctrl_module_pub_.publish(str_msg);

  set_gripper_module_msg_pub_.publish(msg);
  set_position_ctrl_module_msg_pub_.publish(msg);
}

void QNode::sendEnableJointControlMode(std_msgs::String msg)
{
  enable_joint_control_mode_pub_.publish(msg);
}

void QNode::sendEnableTaskSpaceControlMode(std_msgs::String msg)
{
  enable_task_space_control_mode_pub_.publish(msg);
}

void QNode::sendEnableMotionPlanningMode(std_msgs::String msg)
{
  enable_motion_planning_mode_pub_.publish(msg);
}

void QNode::setZeroPosition(std_msgs::String msg)
{
  set_zero_position_pub_.publish(msg);
}

void QNode::setInitPosition(std_msgs::String msg)
{
  set_init_position_pub_.publish(msg);
}

void QNode::getJointPresentPosition(void)
{
  manipulator_x_position_ctrl_module_msgs::GetJointPose srv;

  if (joint_present_position_client_.call(srv))
  {
    manipulator_x_position_ctrl_module_msgs::JointPose msg;

    msg.joint_name = srv.response.position_ctrl_joint_pose.joint_name;
    msg.position = srv.response.position_ctrl_joint_pose.position;

    Q_EMIT updateJointPresentPosition(msg);
  }
}

void QNode::getKinematicsPresentPosition(void)
{
  manipulator_x_position_ctrl_module_msgs::GetKinematicsPose srv;

  if (kinematics_present_pose_client_.call(srv))
  {
    manipulator_x_position_ctrl_module_msgs::KinematicsPose msg;

    msg.group_name = srv.response.position_ctrl_kinematics_pose.group_name;
    msg.pose = srv.response.position_ctrl_kinematics_pose.pose;

    Q_EMIT updateKinematicsPresentPose(msg);
  }
}

void QNode::sendJointGoalPositionMsg(manipulator_x_position_ctrl_module_msgs::JointPose msg)
{
  send_goal_joint_position_pub_.publish(msg);
}

void QNode::sendKinematicsPositionMsg(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg)
{
  set_kinematics_pose_msg_pub_.publish(msg);
}

void QNode::sendMotionPlanningTargetPoseMsg(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg)
{
  set_motion_planning_pose_msg_pub_.publish(msg);
}

void QNode::sendGripperGoalPositionMsg(std_msgs::Float64 msg)
{
  gripper_goal_position_pub_.publish(msg);
}

void QNode::run()
{
  ros::Rate loop_rate(125);

  while ( ros::ok() )
  {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  if (msg->status_msg == "End Trajectory")
  {
    getJointPresentPosition();
    getKinematicsPresentPosition();
  }

  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
	logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "]";

	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << _sender.str() << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace manipulator_x_position_ctrl_module_gui
