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
 *  Created on: Jul 6, 2016
 *      Author: sch
 */

#ifndef manipulator_x_torque_ctrl_gui_MAIN_WINDOW_H
#define manipulator_x_torque_ctrl_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace manipulator_x_torque_ctrl_gui
{

#define deg2rad (M_PI/180.0)
#define rad2deg (180.0/M_PI)

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

  void on_set_mode_button_clicked(bool check);

  void on_load_gain_pushbutton_clicked(bool check);
  void on_save_gain_pushbutton_clicked(bool check);
  void on_set_gain_pushbutton_clicked(bool check);
  void on_zero_gain_pushbutton_clicked(bool check);

  void on_get_pre_value_pushbutton_clicked(bool check);
  void on_send_des_value_pushbutton_clicked(bool check);

  void on_send_des_wrench_pushbutton_clicked(bool check);

  void on_joint_control_checkbox_clicked(bool check);
  void on_force_control_checkbox_clicked(bool check);

  /******************************************
    ** Manual connections
    *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

  void updateJointGainSpinbox(manipulator_x_torque_ctrl_module_msgs::JointGain msg);
  void updateJointPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::JointPose msg);

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;

  std::vector<std::string> joint_name_;

  QList<QAbstractSpinBox *> p_gain_spinbox_;
  QList<QAbstractSpinBox *> i_gain_spinbox_;
  QList<QAbstractSpinBox *> d_gain_spinbox;

  QList<QAbstractSpinBox *> present_joint_angle_spinbox_;
  QList<QAbstractSpinBox *> desired_joint_angle_spinbox_;

  QList<QAbstractSpinBox *> desired_wrench_spinbox_;
};

}  // namespace manipulator_x_torque_ctrl_gui

#endif // manipulator_x_torque_ctrl_gui_MAIN_WINDOW_H
