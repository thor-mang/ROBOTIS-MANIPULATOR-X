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

  present_task_space_position_spinbox_.push_back(ui_.pre_position_x_spinbox);
  present_task_space_position_spinbox_.push_back(ui_.pre_position_y_spinbox);
  present_task_space_position_spinbox_.push_back(ui_.pre_position_z_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_r_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_p_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_y_spinbox);

  desired_task_space_position_spinbox_.push_back(ui_.des_position_x_spinbox);
  desired_task_space_position_spinbox_.push_back(ui_.des_position_y_spinbox);
  desired_task_space_position_spinbox_.push_back(ui_.des_position_z_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_r_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_p_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_y_spinbox);

  kinematics_p_gain_spinbox_.push_back(ui_.position_x_p_gain_spinbox);
  kinematics_p_gain_spinbox_.push_back(ui_.position_y_p_gain_spinbox);
  kinematics_p_gain_spinbox_.push_back(ui_.position_z_p_gain_spinbox);
  kinematics_p_gain_spinbox_.push_back(ui_.orientation_r_p_gain_spinbox);
  kinematics_p_gain_spinbox_.push_back(ui_.orientation_p_p_gain_spinbox);
  kinematics_p_gain_spinbox_.push_back(ui_.orientation_y_p_gain_spinbox);

  kinematics_d_gain_spinbox_.push_back(ui_.position_x_d_gain_spinbox);
  kinematics_d_gain_spinbox_.push_back(ui_.position_y_d_gain_spinbox);
  kinematics_d_gain_spinbox_.push_back(ui_.position_z_d_gain_spinbox);
  kinematics_d_gain_spinbox_.push_back(ui_.orientation_r_d_gain_spinbox);
  kinematics_d_gain_spinbox_.push_back(ui_.orientation_p_d_gain_spinbox);
  kinematics_d_gain_spinbox_.push_back(ui_.orientation_y_d_gain_spinbox);

  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::JointGain>("manipulator_x_torque_ctrl_module_msgs::JointGain");
  QObject::connect(&qnode_,
                   SIGNAL(updateJointGain(manipulator_x_torque_ctrl_module_msgs::JointGain)),
                   this,
                   SLOT(updateJointGainSpinbox(manipulator_x_torque_ctrl_module_msgs::JointGain)));

  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::JointPose>("manipulator_x_torque_ctrl_module_msgs::JointPose");
  QObject::connect(&qnode_,
                   SIGNAL(updateJointPose(manipulator_x_torque_ctrl_module_msgs::JointPose)),
                   this,
                   SLOT(updateJointPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::JointPose)));

  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::KinematicsPose>("manipulator_x_torque_ctrl_module_msgs::KinematicsPose");
  QObject::connect(&qnode_,
                   SIGNAL(updateKinematicsPose(manipulator_x_torque_ctrl_module_msgs::KinematicsPose)),
                   this,
                   SLOT(updateKinematicsPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::KinematicsPose)));

  qRegisterMetaType<manipulator_x_torque_ctrl_module_msgs::KinematicsGain>("manipulator_x_torque_ctrl_module_msgs::KinematicsGain");
  QObject::connect(&qnode_,
                   SIGNAL(updateKinematicsGain(manipulator_x_torque_ctrl_module_msgs::KinematicsGain)),
                   this,
                   SLOT(updateKinematicsGainSpinbox(manipulator_x_torque_ctrl_module_msgs::KinematicsGain)));

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

void MainWindow::on_go_initial_pose_button_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::JointPose msg;

  msg.mov_time = 1.2;

  msg.joint_name.push_back("joint1");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint2");
  msg.position.push_back(-60.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint3");
  msg.position.push_back(-30.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint4");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint5");
  msg.position.push_back(-30.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint6");
  msg.position.push_back(0.0*DEGREE2RADIAN);

  qnode_.sendJointPoseMsg(msg);
}

void MainWindow::on_go_zero_pose_button_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::JointPose msg;

  msg.mov_time = 1.2;

  msg.joint_name.push_back("joint1");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint2");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint3");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint4");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint5");
  msg.position.push_back(0.0*DEGREE2RADIAN);
  msg.joint_name.push_back("joint6");
  msg.position.push_back(0.0*DEGREE2RADIAN);

  qnode_.sendJointPoseMsg(msg);
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

  msg.mov_time = 0.8;

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

void MainWindow::on_get_pre_pose_pushbutton_clicked(bool check)
{
  qnode_.getKinematicsPose();
}

void MainWindow::on_send_des_pose_pushbutton_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::KinematicsPose msg;

  msg.group_name = "arm";
  msg.mov_time = 2.0;

  msg.pose.position.x = ui_.des_position_x_spinbox->value();
  msg.pose.position.y = ui_.des_position_y_spinbox->value();
  msg.pose.position.z = ui_.des_position_z_spinbox->value();

  double roll = ui_.des_orientation_r_spinbox->value() * DEGREE2RADIAN;
  double pitch = ui_.des_orientation_p_spinbox->value() * DEGREE2RADIAN;
  double yaw = ui_.des_orientation_y_spinbox->value() * DEGREE2RADIAN;

  Eigen::Quaterniond quaternion = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  msg.pose.orientation.x = quaternion.x();
  msg.pose.orientation.y = quaternion.y();
  msg.pose.orientation.z = quaternion.z();
  msg.pose.orientation.w = quaternion.w();

  qnode_.sendKinematicsPoseMsg(msg);
}

void MainWindow::on_force_set_gain_pushbutton_clicked(bool check)
{
  manipulator_x_torque_ctrl_module_msgs::KinematicsGain msg;

  msg.pose_value.push_back(0);
  msg.p_gain.push_back(ui_.position_x_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.position_x_d_gain_spinbox->value());

  msg.pose_value.push_back(1);
  msg.p_gain.push_back(ui_.position_y_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.position_y_d_gain_spinbox->value());

  msg.pose_value.push_back(2);
  msg.p_gain.push_back(ui_.position_z_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.position_z_d_gain_spinbox->value());

  msg.pose_value.push_back(3);
  msg.p_gain.push_back(ui_.orientation_r_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.orientation_r_d_gain_spinbox->value());

  msg.pose_value.push_back(4);
  msg.p_gain.push_back(ui_.orientation_p_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.orientation_p_d_gain_spinbox->value());

  msg.pose_value.push_back(5);
  msg.p_gain.push_back(ui_.orientation_y_p_gain_spinbox->value());
  msg.d_gain.push_back(ui_.orientation_y_d_gain_spinbox->value());

  qnode_.sendKinematicsGainMsg(msg);
}

void MainWindow::on_force_save_gain_pushbutton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_gain";

  qnode_.sendSaveForceGainMsg(msg);
}

void MainWindow::on_force_load_gain_pushbutton_clicked(bool check)
{
  qnode_.getKinematicsGain();
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

void MainWindow::updateKinematicsGainSpinbox(manipulator_x_torque_ctrl_module_msgs::KinematicsGain msg)
{
  for (int index=0; index<msg.pose_value.size(); index++)
  {
    ((QDoubleSpinBox *) kinematics_p_gain_spinbox_[msg.pose_value[index]])->setValue(msg.p_gain[msg.pose_value[index]]);
    ((QDoubleSpinBox *) kinematics_d_gain_spinbox_[msg.pose_value[index]])->setValue(msg.d_gain[msg.pose_value[index]]);
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

void MainWindow::updateKinematicsPoseSpinbox(manipulator_x_torque_ctrl_module_msgs::KinematicsPose msg)
{
  Eigen::MatrixXd position = Eigen::MatrixXd::Zero(3,1);
  position.coeffRef(0,0) = msg.pose.position.x;
  position.coeffRef(1,0) = msg.pose.position.y;
  position.coeffRef(2,0) = msg.pose.position.z;

  for (int index=0; index<present_task_space_position_spinbox_.size(); index++)
  {
    ((QDoubleSpinBox *) present_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
//    ((QDoubleSpinBox *) desired_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
  }

  Eigen::Quaterniond quaterion;
  quaterion.x() = msg.pose.orientation.x;
  quaterion.y() = msg.pose.orientation.y;
  quaterion.z() = msg.pose.orientation.z;
  quaterion.w() = msg.pose.orientation.w;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRPY(quaterion);

  for (int index=0; index<present_task_space_orientation_spinbox_.size(); index++)
  {
    ((QDoubleSpinBox *) present_task_space_orientation_spinbox_[index])->setValue(orientation.coeff(index,0) * RADIAN2DEGREE);
//    ((QDoubleSpinBox *) desired_task_space_orientation_spinbox_[index])->setValue(orientation.coeff(index,0) * RADIAN2DEGREE);
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

