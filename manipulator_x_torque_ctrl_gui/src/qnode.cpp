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
 *      Author: sch
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/manipulator_x_torque_ctrl_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_x_torque_ctrl_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/

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

void QNode::sendSetModeMsg(std_msgs::String msg)
{
  set_mode_msg_pub_.publish(msg);
}

void QNode::sendSaveGainMsg(std_msgs::String msg)
{
  save_gain_msg_pub_.publish(msg);
}

void QNode::sendSetGainMsg(manipulator_x_torque_ctrl_module_msgs::JointGain msg)
{
  set_gain_msg_pub_.publish(msg);
}

void QNode::sendJointPoseMsg(manipulator_x_torque_ctrl_module_msgs::JointPose msg)
{
  set_joint_pose_msg_pub_.publish(msg);
}

void QNode::sendWrenchMsg(geometry_msgs::Wrench msg)
{
  set_wrench_msg_pub_.publish(msg);
}

void QNode::enableJointControl(std_msgs::Bool msg)
{
  enable_joint_control_pub_.publish(msg);
}

void QNode::enableForceControl(std_msgs::Bool msg)
{
  enable_force_control_pub_.publish(msg);
}

void QNode::getJointGain()
{
  manipulator_x_torque_ctrl_module_msgs::GetJointGain srv;

  if (get_joint_gain_client_.call(srv))
  {
    manipulator_x_torque_ctrl_module_msgs::JointGain msg;

    msg.joint_name = srv.response.torque_ctrl_gain.joint_name;
    msg.p_gain = srv.response.torque_ctrl_gain.p_gain;
    msg.i_gain = srv.response.torque_ctrl_gain.i_gain;
    msg.d_gain = srv.response.torque_ctrl_gain.d_gain;

    Q_EMIT updateJointGain(msg);
  }
}

void QNode::getJointPose()
{
  manipulator_x_torque_ctrl_module_msgs::GetJointPose srv;

  if (get_joint_pose_client_.call(srv))
  {
    manipulator_x_torque_ctrl_module_msgs::JointPose msg;

    msg.joint_name = srv.response.torque_ctrl_joint_pose.joint_name;
    msg.position = srv.response.torque_ctrl_joint_pose.position;

    Q_EMIT updateJointPose(msg);
  }
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"manipulator_x_torque_ctrl_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/torque_ctrl/set_mode_msg", 0);
  save_gain_msg_pub_ = n.advertise<std_msgs::String>("/robotis/torque_ctrl/save_gain_msg", 0);
  set_gain_msg_pub_ = n.advertise<manipulator_x_torque_ctrl_module_msgs::JointGain>("/robotis/torque_ctrl/set_gain_msg", 0);
  set_joint_pose_msg_pub_ = n.advertise<manipulator_x_torque_ctrl_module_msgs::JointPose>("/robotis/torque_ctrl/set_joint_pose_msg", 0);
  set_wrench_msg_pub_ = n.advertise<geometry_msgs::Wrench>("/robotis/torque_ctrl/set_wrench_msg", 0);

  enable_joint_control_pub_ = n.advertise<std_msgs::Bool>("/robotis/torque_ctrl/enable_joint_control_msg", 0);
  enable_force_control_pub_ = n.advertise<std_msgs::Bool>("/robotis/torque_ctrl/enable_force_control_msg", 0);

  get_joint_gain_client_ = n.serviceClient<manipulator_x_torque_ctrl_module_msgs::GetJointGain>("/robotis/torque_ctrl/get_joint_gain", 0);
  get_joint_pose_client_ = n.serviceClient<manipulator_x_torque_ctrl_module_msgs::GetJointPose>("/robotis/torque_ctrl/get_joint_pose", 0);

  status_msg_sub_ = n.subscribe("/robotis/status", 10, &QNode::statusMsgCallback, this);

  start();
  return true;
}

void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
    log((LogLevel) msg->type, msg->status_msg, msg->module_name);
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

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;

    std::stringstream _sender;
    _sender << "[" << sender << "] ";

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
            logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Error) : {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Fatal) : {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace manipulator_x_torque_ctrl_gui
