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
 *  Created on: Jan 13, 2017
 *      Author: sch, Darby Lim
 */

#include <ros/ros.h>

#include <pthread.h>

#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "manipulator_x_position_ctrl_module_msgs/KinematicsPose.h"

ros::Publisher status_msg_pub;
ros::Publisher execute_planned_path_msg_pub;
pthread_t trajectory_generate;
manipulator_x_position_ctrl_module_msgs::KinematicsPose kinematics_pose_msg;

void publishStatusMsg(unsigned int type, std::string msg);

void* plan_trajectory_proc(void* arg)
{
  static moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup::Plan my_plan;

  bool success;

  /* ----- set planning time ----- */

  group.setPlanningTime(7.0);

  /* set start state */
  group.setStartState(*group.getCurrentState());

  /* set target state */
  geometry_msgs::Pose target_pose;

  target_pose.position.x = kinematics_pose_msg.pose.position.x;
  target_pose.position.y = kinematics_pose_msg.pose.position.y;
  target_pose.position.z = kinematics_pose_msg.pose.position.z;

  target_pose.orientation.x = kinematics_pose_msg.pose.orientation.x;
  target_pose.orientation.y = kinematics_pose_msg.pose.orientation.y;
  target_pose.orientation.z = kinematics_pose_msg.pose.orientation.z;
  target_pose.orientation.w = kinematics_pose_msg.pose.orientation.w;

  group.setPoseTarget( target_pose );

  /* motion planning */
  success = group.plan( my_plan );

  if ( success == true )
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Planning Success");

    std_msgs::String msg;
    msg.data = "execute";

    execute_planned_path_msg_pub.publish( msg );
  }
  else
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Planning Fail");

    std_msgs::String msg;
    msg.data = "fail";

    execute_planned_path_msg_pub.publish( msg );
  }
}

void motionPlanningTargetPoseMsgCallback( const manipulator_x_position_ctrl_module_msgs::KinematicsPose::ConstPtr& msg )
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Planning Start");

  kinematics_pose_msg = *msg;

  pthread_create( &trajectory_generate , NULL , plan_trajectory_proc , NULL );

  return;
}

void publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Manipulator_x4_planning";
  status_msg.status_msg = msg;

  status_msg_pub.publish(status_msg);
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "manipulator_x4_planning" );
  ros::NodeHandle nh("~");

  status_msg_pub = nh.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 10);
  execute_planned_path_msg_pub = nh.advertise<std_msgs::String>("/robotis/manipulator_x4/position_ctrl/execute_planned_path", 10);

  ros::Subscriber kinematics_pose_msg_sub = nh.subscribe("/robotis/manipulator_x4/position_ctrl/motion_planning_target_pose", 10,
                                                         motionPlanningTargetPoseMsgCallback);

  ros::spin();

  return 0;
}
