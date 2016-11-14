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
#ifndef MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_MAIN_WINDOW_H
#define MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "robotis_math/robotis_math.h"
#endif

namespace manipulator_x_position_ctrl_module_gui
{
#define JOINT_CONTROL (0)
#define TASK_SPACE_CONTROL (1)

class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

 public Q_SLOTS:
	void on_actionAbout_triggered();

  void updateLoggingView(); // no idea why this can't connect automatically
  void on_set_control_mode_pushButton_clicked(bool check);
  void on_send_goal_position_pushButton_clicked(bool check);
  void on_zero_position_pushButton_clicked(bool check);
  void on_init_position_pushButton_clicked(bool check);

  void updateJointPresentPoseLineEdit(manipulator_x_position_ctrl_module_msgs::JointPose msg);
  void changeControlMode(int index);

 private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;
};

}  // namespace manipulator_x_position_ctrl_module_gui

#endif // MANIPULATOR_X_POSITION_CTRL_MODUEL_GUI_MAIN_WINDOW_H
