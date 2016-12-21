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
 * gazebo_grip_pub.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch, Darby Lim
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher grip_joint_pub;

void gripJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg;

  grip_joint_msg.data = msg->data;
  grip_joint_pub.publish(grip_joint_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_manipulator_x4_gripper_publisher");
  ros::NodeHandle nh("~");

  grip_joint_pub = nh.advertise<std_msgs::Float64>("/gazebo/robotis/manipulator_x4/grip_joint_sub_position/command", 5);

  ros::Subscriber grip_joint_sub = nh.subscribe("/gazebo/robotis/manipulator_x4/grip_joint_position/command", 5, gripJointCallback);

  ros::spin();

  return 0;
}
