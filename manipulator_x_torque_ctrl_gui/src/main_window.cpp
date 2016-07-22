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
 * main_window.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: sch
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/manipulator_x_torque_ctrl_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_x_torque_ctrl_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode_(argc,argv)
{
  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");
  joint_name_.push_back("joint5");
  joint_name_.push_back("joint6");

  p_gain_spinbox_.push_back(ui_.joint1_p_gain_spinbox);
  p_gain_spinbox_.push_back(ui_.joint2_p_gain_spinbox);
  p_gain_spinbox_.push_back(ui_.joint3_p_gain_spinbox);
  p_gain_spinbox_.push_back(ui_.joint4_p_gain_spinbox);
  p_gain_spinbox_.push_back(ui_.joint5_p_gain_spinbox);
  p_gain_spinbox_.push_back(ui_.joint6_p_gain_spinbox);

  i_gain_spinbox_.push_back(ui_.joint1_i_gain_spinbox);
  i_gain_spinbox_.push_back(ui_.joint2_i_gain_spinbox);
  i_gain_spinbox_.push_back(ui_.joint3_i_gain_spinbox);
  i_gain_spinbox_.push_back(ui_.joint4_i_gain_spinbox);
  i_gain_spinbox_.push_back(ui_.joint5_i_gain_spinbox);
  i_gain_spinbox_.push_back(ui_.joint6_i_gain_spinbox);

  d_gain_spinbox.push_back(ui_.joint1_d_gain_spinbox);
  d_gain_spinbox.push_back(ui_.joint2_d_gain_spinbox);
  d_gain_spinbox.push_back(ui_.joint3_d_gain_spinbox);
  d_gain_spinbox.push_back(ui_.joint4_d_gain_spinbox);
  d_gain_spinbox.push_back(ui_.joint5_d_gain_spinbox);
  d_gain_spinbox.push_back(ui_.joint6_d_gain_spinbox);

  present_joint_angle_spinbox_.push_back(ui_.joint1_pre_angle_spinbox);
  present_joint_angle_spinbox_.push_back(ui_.joint2_pre_angle_spinbox);
  present_joint_angle_spinbox_.push_back(ui_.joint3_pre_angle_spinbox);
  present_joint_angle_spinbox_.push_back(ui_.joint4_pre_angle_spinbox);
  present_joint_angle_spinbox_.push_back(ui_.joint5_pre_angle_spinbox);
  present_joint_angle_spinbox_.push_back(ui_.joint6_pre_angle_spinbox);

  desired_joint_angle_spinbox_.push_back(ui_.joint1_des_angle_spinbox);
  desired_joint_angle_spinbox_.push_back(ui_.joint2_des_angle_spinbox);
  desired_joint_angle_spinbox_.push_back(ui_.joint3_des_angle_spinbox);
  desired_joint_angle_spinbox_.push_back(ui_.joint4_des_angle_spinbox);
  desired_joint_angle_spinbox_.push_back(ui_.joint5_des_angle_spinbox);
  desired_joint_angle_spinbox_.push_back(ui_.joint6_des_angle_spinbox);

  desired_wrench_spinbox_.push_back(ui_.des_force_x_spinbox);
  desired_wrench_spinbox_.push_back(ui_.des_force_y_spinbox);
  desired_wrench_spinbox_.push_back(ui_.des_force_z_spinbox);
  desired_wrench_spinbox_.push_back(ui_.des_torque_x_spinbox);
  desired_wrench_spinbox_.push_back(ui_.des_torque_y_spinbox);
  desired_wrench_spinbox_.push_back(ui_.des_torque_z_spinbox);


  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::JointGain>("manipulator_x_torque_ctrl_module_msgs::JointGain");
  QObject::connect(&qnode_, SIGNAL(updateJointGain(manipulator_x_torque_ctrl_module_msgs::JointGain)), this, SLOT(updateJointGainSpinbox(manipulator_x_torque_ctrl_module_msgs::JointGain)));

  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::JointPose>("manipulator_x_torque_ctrl_module_msgs::JointPose");
  QObject::connect(&qnode_, SIGNAL(updateJointPose(manipulator_x_torque_ctrl_module_msgs::JointPose)), this, SLOT(updateJointPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::JointPose)));

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_set_torque_control_mode_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "set_mode";

  qnode_.sendSetModeMsg(msg);
}

void MainWindow::on_set_position_control_mode_button_clicked(bool check)
{

}

void MainWindow::on_go_initial_pose_button_clicked(bool check)
{


}

void MainWindow::on_load_gain_pushbutton_clicked(bool check)
{
  qnode_.getJointGain();
}

void MainWindow::on_save_gain_pushbutton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_gain";

  qnode_.sendSaveGainMsg(msg);
}

void MainWindow::on_set_gain_pushbutton_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::JointGain msg;

  for (int it=0; it<joint_name_.size(); it++)
  {
    msg.joint_name.push_back(joint_name_[it]);
    msg.p_gain.push_back(((QDoubleSpinBox *) p_gain_spinbox_[it])->value());
    msg.i_gain.push_back(((QDoubleSpinBox *) i_gain_spinbox_[it])->value());
    msg.d_gain.push_back(((QDoubleSpinBox *) d_gain_spinbox[it])->value());
  }

  qnode_.sendSetGainMsg(msg);
}

void MainWindow::on_zero_gain_pushbutton_clicked(bool check)
{
  for (int name_index=0; name_index<joint_name_.size(); name_index++)
  {
    ((QDoubleSpinBox *) p_gain_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) i_gain_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) d_gain_spinbox[name_index])->setValue(0.0);
  }
}

void MainWindow::on_get_pre_value_pushbutton_clicked(bool check)
{
  qnode_.getJointPose();
}

void MainWindow::on_send_des_value_pushbutton_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::JointPose msg;

  for (int it=0; it<joint_name_.size(); it++)
  {
    msg.joint_name.push_back(joint_name_[it]);
    msg.position.push_back(((QDoubleSpinBox *) desired_joint_angle_spinbox_[it])->value()*DEGREE2RADIAN);
  }

  qnode_.sendJointPoseMsg(msg);
}

void MainWindow::on_send_des_wrench_pushbutton_clicked(bool check)
{
  geometry_msgs::Wrench msg;

  msg.force.x = ((QDoubleSpinBox *) desired_wrench_spinbox_[0])->value();
  msg.force.y = ((QDoubleSpinBox *) desired_wrench_spinbox_[1])->value();
  msg.force.z = ((QDoubleSpinBox *) desired_wrench_spinbox_[2])->value();

  msg.torque.x = ((QDoubleSpinBox *) desired_wrench_spinbox_[3])->value();
  msg.torque.y = ((QDoubleSpinBox *) desired_wrench_spinbox_[4])->value();
  msg.torque.z = ((QDoubleSpinBox *) desired_wrench_spinbox_[5])->value();

  qnode_.sendWrenchMsg(msg);
}

void MainWindow::on_joint_control_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.joint_control_checkbox->isChecked();

  if (msg.data == true)
    ui_.force_control_checkbox->setChecked(false);

  qnode_.enableJointControl(msg);
}

void MainWindow::on_force_control_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.force_control_checkbox->isChecked();

  if (msg.data == true)
    ui_.joint_control_checkbox->setChecked(false);

  qnode_.enableForceControl(msg);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::updateJointGainSpinbox(manipulator_x_torque_ctrl_module_msgs::JointGain msg)
{
  for (int name_index=0; name_index<msg.joint_name.size(); name_index++)
  {
    for (int joint_index=0; joint_index<joint_name_.size(); joint_index++)
    {
      if (msg.joint_name[joint_index] == joint_name_[name_index])
      {
        ((QDoubleSpinBox *) p_gain_spinbox_[name_index])->setValue(msg.p_gain[joint_index]);
        ((QDoubleSpinBox *) i_gain_spinbox_[name_index])->setValue(msg.i_gain[joint_index]);
        ((QDoubleSpinBox *) d_gain_spinbox[name_index])->setValue(msg.d_gain[joint_index]);
        break;
      }
    }
  }
}

void MainWindow::updateJointPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::JointPose msg)
{
  for (int name_index=0; name_index<msg.joint_name.size(); name_index++)
  {
    for (int joint_index=0; joint_index<joint_name_.size(); joint_index++)
    {
      if (msg.joint_name[joint_index] == joint_name_[name_index])
      {
        ((QDoubleSpinBox *) present_joint_angle_spinbox_[name_index])->setValue(msg.position[joint_index]*RADIAN2DEGREE);
        break;
      }
    }
  }
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<p>Copyright Robotis</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace manipulator_x_torque_ctrl_gui

