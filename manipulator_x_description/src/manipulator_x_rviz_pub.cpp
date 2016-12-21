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
 * manipulator_x_rviz_pub.cpp
 *
 *  Created on: Jul 8, 2016
 *      Author: sch, Darby Lim
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher present_joint_states_pub;
ros::Publisher goal_joint_states_pub;

void present_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
  sensor_msgs::JointState _present_msg;

  for ( int _index = 0 ; _index < msg->name.size(); _index++ )
  {
    if ( msg->name[ _index ] == "grip_joint" )
    {
      _present_msg.name.push_back( msg->name[ _index ] );
      _present_msg.position.push_back( msg->position[ _index ] * 0.01);
      _present_msg.name.push_back("grip_joint_sub");
      _present_msg.position.push_back(_present_msg.position[ _index ]);
    }
    else
    {
      _present_msg.name.push_back( msg->name[ _index ] );
      _present_msg.position.push_back( msg->position[ _index ] );
    }
  }
  present_joint_states_pub.publish( _present_msg );
}

void goal_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
  sensor_msgs::JointState _goal_msg;

  for ( int _index = 0 ; _index < msg->name.size(); _index++ )
  {
    if ( msg->name[ _index ] == "grip_joint" )
    {
      _goal_msg.name.push_back( msg->name[ _index ] );
      _goal_msg.position.push_back( msg->position[ _index ] * 0.01);
      _goal_msg.name.push_back("grip_joint_sub");
      _goal_msg.position.push_back(_goal_msg.position[ _index ]);
    }
    else
    {
      _goal_msg.name.push_back( msg->name[ _index ] );
      _goal_msg.position.push_back( msg->position[ _index ] );
    }
  }
  goal_joint_states_pub.publish( _goal_msg );
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "manipulator_x4_publisher" );
  ros::NodeHandle nh("~");

  present_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator_x4/present_joint_states", 0);
  goal_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator_x4/goal_joint_states", 0);

  ros::Subscriber present_joint_states_sub = nh.subscribe("/robotis/present_joint_states", 5, present_joint_states_callback);
  ros::Subscriber goal_joint_states_sub = nh.subscribe("/robotis/goal_joint_states", 5, goal_joint_states_callback);

  ros::spin();

  return 0;
}



