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
 * main_window.hpp
 *
 *  Created on: Jul 22, 2016
 *      Author: sch
 */

#ifndef manipulator_x_position_ctrl_gui_MAIN_WINDOW_H
#define manipulator_x_position_ctrl_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "robotis_math/robotis_math.h"

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace manipulator_x_position_ctrl_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();

  void on_set_position_control_mode_button_clicked(bool check);
  void on_go_zero_pose_button_clicked(bool check);
  void on_go_initial_pose_button_clicked(bool check);

  void on_get_pre_value_pushbutton_clicked(bool check);
  void on_send_des_value_pushbutton_clicked(bool check);

  void on_get_pre_pose_pushbutton_clicked(bool check);
  void on_send_des_pose_pushbutton_clicked(bool check);

  void on_joint_space_control_checkbox_clicked(bool check);
  void on_task_space_control_checkbox_clicked(bool check);
  void on_motion_planning_checkbox_clicked(bool check);

  /******************************************
    ** Manual connections
    *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

  void updateJointPoseSpinbox(manipulator_x_position_ctrl_module_msgs::JointPose msg);
  void updateKinematicsPoseSpinbox(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg);

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;

  std::vector<std::string> joint_name_;

  QList<QAbstractSpinBox *> present_joint_angle_spinbox_;
  QList<QAbstractSpinBox *> desired_joint_angle_spinbox_;

  QList<QAbstractSpinBox *> present_task_space_position_spinbox_;
  QList<QAbstractSpinBox *> present_task_space_orientation_spinbox_;

  QList<QAbstractSpinBox *> desired_task_space_position_spinbox_;
  QList<QAbstractSpinBox *> desired_task_space_orientation_spinbox_;
};

}  // namespace manipulator_x_position_ctrl_gui

#endif // manipulator_x_position_ctrl_gui_MAIN_WINDOW_H
