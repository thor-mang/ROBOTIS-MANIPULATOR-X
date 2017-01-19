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
 * motion_planning.h
 *
 *  Created on: Jan 17, 2017
 *      Author: sch, Darby Lim
 */

#ifndef MOTION_PLANNING_TOOL_H
#define MOTION_PLANNING_TOOL_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

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

#include "robotis_math/robotis_math.h"

namespace motion_planning_tool
{
#define MAX_JOINT_NUM  (4)

class MotionPlanningTool
{
 public:
  MotionPlanningTool();
  ~MotionPlanningTool();

  int points_; // planned number of via-points

  ros::Duration time_from_start_; // planned movement time

  Eigen::MatrixXd display_planned_path_positions_; // planned position trajectory
  Eigen::MatrixXd display_planned_path_velocities_; // planned velocity trajectory
  Eigen::MatrixXd display_planned_path_accelerations_; // planned acceleration trajectory

  moveit_msgs::DisplayTrajectory moveit_msg_;

  // initialization
  boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;

  robot_state::JointModelGroup* arm_joint_model_group_;
  std::vector<std::string> arm_joint_names_;

  std::vector<double> _arm_joint_values_ , arm_joint_values_ ;

  boost::shared_ptr<planning_scene::PlanningScene> planning_scene_;

  // moveit path planning
  boost::shared_ptr<moveit::planning_interface::MoveGroup> arm_planning_group_;

  void init(std::string description);
};

}

#endif /* MOTION_PLANNING_TOOL_H */
