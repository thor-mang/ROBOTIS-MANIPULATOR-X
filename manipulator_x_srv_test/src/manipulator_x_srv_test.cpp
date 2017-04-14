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
 * manipulator_x_srv_test.cpp
 *
 *  Created on: April 14, 2017
 *      Author: sch
 */

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>

#include "manipulator_x_position_ctrl_module_msgs/JointPoseToKinematicsPose.h"
#include "manipulator_x_position_ctrl_module_msgs/KinematicsPoseToJointPose.h"

ros::ServiceClient joint_pose_to_kinematics_pose_client;
ros::ServiceClient kinematics_pose_to_joint_pose_client;


void jointPoseToKinematicsPose()
{
  manipulator_x_position_ctrl_module_msgs::JointPoseToKinematicsPose srv;

//  ROS_INFO("---Request---");
//  double joint1_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint1");
//  srv.request.joint_pose.position.push_back(joint1_angle);
//  ROS_INFO("joint1 : %f", joint1_angle);
//  double joint2_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint2");
//  srv.request.joint_pose.position.push_back(joint2_angle);
//  ROS_INFO("joint2 : %f", joint2_angle);
//  double joint3_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint3");
//  srv.request.joint_pose.position.push_back(joint3_angle);
//  ROS_INFO("joint3 : %f", joint3_angle);
//  double joint4_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint4");
//  srv.request.joint_pose.position.push_back(joint4_angle);
//  ROS_INFO("joint4 : %f", joint4_angle);
//  double joint5_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint5");
//  srv.request.joint_pose.position.push_back(joint5_angle);
//  ROS_INFO("joint5 : %f", joint5_angle);
//  double joint6_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint6");
//  srv.request.joint_pose.position.push_back(joint6_angle);
//  ROS_INFO("joint6 : %f", joint6_angle);
//  double joint7_angle = 0.0 ; // rad
//  srv.request.joint_pose.name.push_back("joint7");
//  srv.request.joint_pose.position.push_back(joint7_angle);
//  ROS_INFO("joint7 : %f", joint7_angle);

  ROS_INFO("-----Joint Pose To Kinematics Pose -----");

  ROS_INFO("---Request---");
  double joint1_angle = 0.0 ; // rad
  srv.request.joint_pose.name.push_back("joint1");
  srv.request.joint_pose.position.push_back(joint1_angle);
  ROS_INFO("joint1 : %f", joint1_angle);
  double joint2_angle = -74.41 * M_PI / 180.0 ; // rad
  srv.request.joint_pose.name.push_back("joint2");
  srv.request.joint_pose.position.push_back(joint2_angle);
  ROS_INFO("joint2 : %f", joint2_angle);
  double joint3_angle = 0.0 ; // rad
  srv.request.joint_pose.name.push_back("joint3");
  srv.request.joint_pose.position.push_back(joint3_angle);
  ROS_INFO("joint3 : %f", joint3_angle);
  double joint4_angle = -18.78 * M_PI / 180.0 ; // rad
  srv.request.joint_pose.name.push_back("joint4");
  srv.request.joint_pose.position.push_back(joint4_angle);
  ROS_INFO("joint4 : %f", joint4_angle);
  double joint5_angle = 0.0 ; // rad
  srv.request.joint_pose.name.push_back("joint5");
  srv.request.joint_pose.position.push_back(joint5_angle);
  ROS_INFO("joint5 : %f", joint5_angle);
  double joint6_angle = -55.63 * M_PI / 180.0 ; // rad
  srv.request.joint_pose.name.push_back("joint6");
  srv.request.joint_pose.position.push_back(joint6_angle);
  ROS_INFO("joint6 : %f", joint6_angle);
  double joint7_angle = 0.0; // rad
  srv.request.joint_pose.name.push_back("joint7");
  srv.request.joint_pose.position.push_back(joint7_angle);
  ROS_INFO("joint7 : %f", joint7_angle);

  geometry_msgs::Pose msg;

  if (joint_pose_to_kinematics_pose_client.call(srv))
    msg = srv.response.kinematics_pose;

  ROS_INFO("---Respose---");
  ROS_INFO("position x : %f", msg.position.x);
  ROS_INFO("position y : %f", msg.position.y);
  ROS_INFO("position z : %f", msg.position.z);

  ROS_INFO("orientation x : %f", msg.orientation.x);
  ROS_INFO("orientation y : %f", msg.orientation.y);
  ROS_INFO("orientation z : %f", msg.orientation.z);
  ROS_INFO("orientation w : %f", msg.orientation.w);
}

void kinematicsPoseToJointPose()
{
  manipulator_x_position_ctrl_module_msgs::KinematicsPoseToJointPose srv;

  srv.request.kinematics_pose.position.x = 0.15;
  srv.request.kinematics_pose.position.y = 0.0;
  srv.request.kinematics_pose.position.z = 0.22;

  srv.request.kinematics_pose.orientation.x = 0.0;
  srv.request.kinematics_pose.orientation.y = 0.0;
  srv.request.kinematics_pose.orientation.z = 0.0;
  srv.request.kinematics_pose.orientation.w = 1.0;

  ROS_INFO("-----Joint Pose To Kinematics Pose -----");

  ROS_INFO("---Request---");
  ROS_INFO("position x : %f", srv.request.kinematics_pose.position.x);
  ROS_INFO("position y : %f", srv.request.kinematics_pose.position.y);
  ROS_INFO("position z : %f", srv.request.kinematics_pose.position.z);

  ROS_INFO("orientation x : %f", srv.request.kinematics_pose.orientation.x);
  ROS_INFO("orientation y : %f", srv.request.kinematics_pose.orientation.y);
  ROS_INFO("orientation z : %f", srv.request.kinematics_pose.orientation.z);
  ROS_INFO("orientation w : %f", srv.request.kinematics_pose.orientation.w);

  sensor_msgs::JointState msg;

  if (kinematics_pose_to_joint_pose_client.call(srv))
    msg = srv.response.joint_pose;

  ROS_INFO("---Respose---");
  for (int i=0; i<msg.name.size(); i++)
  {
    std::string joint_name = msg.name[i];
    double joint_angle = msg.position[i];

    ROS_INFO("%s : %f", joint_name.c_str(), joint_angle);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator_x_srv_test");
  ros::NodeHandle nh("~");

  joint_pose_to_kinematics_pose_client =
      nh.serviceClient<manipulator_x_position_ctrl_module_msgs::JointPoseToKinematicsPose>("/robotis/joint_pose_to_kinematics_pose", 0);
  kinematics_pose_to_joint_pose_client =
      nh.serviceClient<manipulator_x_position_ctrl_module_msgs::KinematicsPoseToJointPose>("/robotis/kinematics_pose_to_joint_pose", 0);

  jointPoseToKinematicsPose();

  kinematicsPoseToJointPose();

  ros::spin();

  return 0;
}
