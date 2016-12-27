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
 *      Author: sch, Darby Lim
 */

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/manipulator_x_position_ctrl_module_gui/main_window.hpp"

namespace manipulator_x_position_ctrl_module_gui
{
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode_(argc,argv)
{
  ui_.setupUi(this);
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  qRegisterMetaType<manipulator_x_position_ctrl_module_msgs::JointPose>("manipulator_x_position_ctrl_module_msgs::JointPose");
  QObject::connect(&qnode_, SIGNAL(updateJointPresentPosition(manipulator_x_position_ctrl_module_msgs::JointPose)),
                   this, SLOT(updateJointPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::JointPose)));

  qRegisterMetaType<manipulator_x_position_ctrl_module_msgs::KinematicsPose>("manipulator_x_position_ctrl_module_msgs::KinematicsPose");
  QObject::connect(&qnode_, SIGNAL(updateKinematicsPresentPosition(manipulator_x_position_ctrl_module_msgs::KinematicsPose)),
                   this, SLOT(updateKinematicsPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::KinematicsPose)));

 // ui_.gripper_goal_position_horizontalSlider->
  QObject::connect(ui_.gripper_goal_position_horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(changeGripperPosition(int)));

  QObject::connect(ui_.manipulator_x4_tabWidget, SIGNAL(currentChanged(int)), this, SLOT(changeControlMode(int)));
  qnode_.init();
}

MainWindow::~MainWindow() {}

void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::on_set_control_module_pushButton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "set_module";

  qnode_.sendSetModuleMsg(msg);

  control_mode_ = JOINT_SPACE_CONTROL;
}

void MainWindow::on_zero_position_pushButton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "init_position";

  qnode_.setZeroPosition(msg);

  ui_.manipulator_x4_tabWidget->setCurrentIndex(JOINT_SPACE_CONTROL);
}

void MainWindow::on_init_position_pushButton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "zero_position";

  qnode_.setInitPosition(msg);

  ui_.manipulator_x4_tabWidget->setCurrentIndex(JOINT_SPACE_CONTROL);
}

void MainWindow::changeControlMode(int index)
{
  std_msgs::String str_msg;

  control_mode_ = index;

  if (control_mode_ == JOINT_SPACE_CONTROL)
  {
    str_msg.data = "set_joint_control_mode";
    qnode_.sendEnableJointControlMode(str_msg);

    ui_.send_goal_position_pushButton->setText("Send Goal Joint Position");
  }
  else if (control_mode_ == TASK_SPACE_CONTROL)
  {
    str_msg.data = "set_task_space_control_mode";
    qnode_.sendEnableTaskSpaceControlMode(str_msg);

    ui_.send_goal_position_pushButton->setText("Send Kinematics Position");
  }
}

void MainWindow::on_gripper_goal_position_toggleButton_clicked(bool check)
{
  std_msgs::Float64 position;


  if (ui_.gripper_goal_position_toggleButton->isChecked())
  {
    ui_.gripper_goal_position_toggleButton->setText("Gripper Closed");

    position.data = 0.0 * DEGREE2RADIAN;
    qnode_.sendGripperGoalPositionMsg(position);

    ui_.gripper_present_position_lineEdit->setText(QString::number(0.0));
    ui_.gripper_goal_position_horizontalSlider->setSliderPosition(0);
  }
  else
  {
    ui_.gripper_goal_position_toggleButton->setText("Gripper Open");

    position.data = 170.0 * DEGREE2RADIAN;
    qnode_.sendGripperGoalPositionMsg(position);

    ui_.gripper_present_position_lineEdit->setText(QString::number(140.0));
    ui_.gripper_goal_position_horizontalSlider->setSliderPosition(140);
  }
}

void MainWindow::changeGripperPosition(int position)
{
  std_msgs::Float64 gripper_position;
  gripper_position.data = position * DEGREE2RADIAN;

  qnode_.sendGripperGoalPositionMsg(gripper_position);

  ui_.gripper_present_position_lineEdit->setText(QString::number(position));
}

void MainWindow::on_send_goal_position_pushButton_clicked(bool check)
{
  if (control_mode_ == JOINT_SPACE_CONTROL)
  {
    manipulator_x_position_ctrl_module_msgs::JointPose msg;

    msg.move_time = 1.5;
    msg.joint_name.push_back("joint1");
    msg.joint_name.push_back("joint2");
    msg.joint_name.push_back("joint3");
    msg.joint_name.push_back("joint4");

    msg.position.push_back(ui_.joint1_goal_position_doubleSpinBox->value()*DEGREE2RADIAN);
    msg.position.push_back(ui_.joint2_goal_position_doubleSpinBox->value()*DEGREE2RADIAN);
    msg.position.push_back(ui_.joint3_goal_position_doubleSpinBox->value()*DEGREE2RADIAN);
    msg.position.push_back(ui_.joint4_goal_position_doubleSpinBox->value()*DEGREE2RADIAN);

    qnode_.sendJointGoalPositionMsg(msg);
  }
  else if (control_mode_ == TASK_SPACE_CONTROL)
  {
    manipulator_x_position_ctrl_module_msgs::KinematicsPose msg;

    msg.group_name = "arm";
    msg.move_time = 1.5;

    msg.position.position.x = ui_.goal_x_doubleSpinBox->value();
    msg.position.position.y = ui_.goal_y_doubleSpinBox->value();
    msg.position.position.z = ui_.goal_z_doubleSpinBox->value();

    double roll  = ui_.goal_roll_doubleSpinBox->value() * DEGREE2RADIAN;
    double pitch = ui_.goal_pitch_doubleSpinBox->value() * DEGREE2RADIAN;
    double yaw   = ui_.goal_yaw_doubleSpinBox->value() * DEGREE2RADIAN;

    Eigen::Quaterniond quaternion = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

    msg.position.orientation.x = quaternion.x();
    msg.position.orientation.y = quaternion.y();
    msg.position.orientation.z = quaternion.z();
    msg.position.orientation.w = quaternion.w();

    qnode_.sendKinematicsPositionMsg(msg);
  }
}

void MainWindow::updateJointPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::JointPose msg)
{
  ui_.joint1_present_position_lineEdit->setText(QString::number(msg.position[0]*RADIAN2DEGREE, 1, 1));
  ui_.joint2_present_position_lineEdit->setText(QString::number(msg.position[1]*RADIAN2DEGREE, 1, 1));
  ui_.joint3_present_position_lineEdit->setText(QString::number(msg.position[2]*RADIAN2DEGREE, 1, 1));
  ui_.joint4_present_position_lineEdit->setText(QString::number(msg.position[3]*RADIAN2DEGREE, 1, 1));

  ui_.joint1_goal_position_doubleSpinBox->setValue(msg.position[0]*RADIAN2DEGREE);
  ui_.joint2_goal_position_doubleSpinBox->setValue(msg.position[1]*RADIAN2DEGREE);
  ui_.joint3_goal_position_doubleSpinBox->setValue(msg.position[2]*RADIAN2DEGREE);
  ui_.joint4_goal_position_doubleSpinBox->setValue(msg.position[3]*RADIAN2DEGREE);
}

void MainWindow::updateKinematicsPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg)
{
  ui_.present_x_lineEdit->setText(QString::number(msg.position.position.x, 'f', 4));
  ui_.present_y_lineEdit->setText(QString::number(msg.position.position.y, 'f', 4));
  ui_.present_z_lineEdit->setText(QString::number(msg.position.position.z, 'f', 4));

  ui_.goal_x_doubleSpinBox->setValue(msg.position.position.x);
  ui_.goal_y_doubleSpinBox->setValue(msg.position.position.y);
  ui_.goal_z_doubleSpinBox->setValue(msg.position.position.z);

  Eigen::Quaterniond quaterion;
  quaterion.x() = msg.position.orientation.x;
  quaterion.y() = msg.position.orientation.y;
  quaterion.z() = msg.position.orientation.z;
  quaterion.w() = msg.position.orientation.w;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRPY(quaterion);

  ui_.present_roll_lineEdit->setText(QString::number(orientation.coeff(0,0) * RADIAN2DEGREE, 'f', 4));
  ui_.present_pitch_lineEdit->setText(QString::number(orientation.coeff(1,0) * RADIAN2DEGREE, 'f', 4));
  ui_.present_yaw_lineEdit->setText(QString::number(orientation.coeff(2,0) * RADIAN2DEGREE, 'f', 4));

  ui_.goal_roll_doubleSpinBox->setValue(orientation.coeff(0,0) * RADIAN2DEGREE);
  ui_.goal_pitch_doubleSpinBox->setValue(orientation.coeff(1,0) * RADIAN2DEGREE);
  ui_.goal_yaw_doubleSpinBox->setValue(orientation.coeff(2,0) * RADIAN2DEGREE);
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<p>Copyright ROBOTIS</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace manipulator_x_position_ctrl_module_gui

