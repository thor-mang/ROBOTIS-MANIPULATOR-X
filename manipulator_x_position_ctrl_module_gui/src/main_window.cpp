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

/* Author: Taehoon Lim (Darby) */

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
  QObject::connect(&qnode_, SIGNAL(updateJointPresentPose(manipulator_x_position_ctrl_module_msgs::JointPose)),
                   this, SLOT(updateJointPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::JointPose)));



  qnode_.init();
}

MainWindow::~MainWindow() {}

void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::on_set_control_mode_pushButton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "set_position_control_mode";

  qnode_.sendSetModeMsg(msg);
}

void MainWindow::on_get_present_position_pushButton_clicked(bool check)
{
  qnode_.getJointPresentPosition();
}

void MainWindow::on_send_goal_position_pushButton_clicked(bool check)
{
  manipulator_x_position_ctrl_module_msgs::JointPose msg;

  msg.mov_time = 1.5;
  msg.position.push_back(ui_.joint1_des_pos_doubleSpinBox->value()*DEGREE2RADIAN);
  msg.position.push_back(ui_.joint2_des_pos_doubleSpinBox->value()*DEGREE2RADIAN);
  msg.position.push_back(ui_.joint3_des_pos_doubleSpinBox->value()*DEGREE2RADIAN);
  msg.position.push_back(ui_.joint4_des_pos_doubleSpinBox->value()*DEGREE2RADIAN);

  qnode_.sendJointGoalPositionMsg(msg);
}

void MainWindow::updateJointPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::JointPose msg)
{
  ui_.joint1_pre_pos_lineEdit->setText(QString::number(msg.position[0]*RADIAN2DEGREE, 1, 1));
  ui_.joint2_pre_pos_lineEdit->setText(QString::number(msg.position[1]*RADIAN2DEGREE, 1, 1));
  ui_.joint3_pre_pos_lineEdit->setText(QString::number(msg.position[2]*RADIAN2DEGREE, 1, 1));
  ui_.joint4_pre_pos_lineEdit->setText(QString::number(msg.position[3]*RADIAN2DEGREE, 1, 1));
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

