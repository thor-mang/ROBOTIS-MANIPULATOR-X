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
 * motion_planning.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: sch, Darby Lim
 */

#include "manipulator_x_position_ctrl_module/motion_planning_tool.h"

using namespace motion_planning_tool;

MotionPlanningTool::MotionPlanningTool()
{
  points_ = 10;

  time_from_start_ = ros::Duration(10.0); // movement duration

  display_planned_path_positions_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM);     // positions of planned path
  display_planned_path_velocities_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM);    // positions of planned path
  display_planned_path_accelerations_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM); // positions of planned path

  // initialization
  robot_model_loader_.reset();
  planning_scene_.reset();
}

MotionPlanningTool::~MotionPlanningTool(){}

void MotionPlanningTool::init(std::string description)
{
  // initialization
  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader(description));
  kinematic_model_ = robot_model_loader_->getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();

  arm_joint_model_group_ 	= 	kinematic_model_->getJointModelGroup("arm");
  arm_joint_names_        = 	arm_joint_model_group_->getJointModelNames();

  planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
}


