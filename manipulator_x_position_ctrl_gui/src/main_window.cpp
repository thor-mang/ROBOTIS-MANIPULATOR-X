/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/manipulator_x_position_ctrl_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_x_position_ctrl_gui {

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

  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<manipulator_x_position_ctrl_module_msgs::JointPose>("manipulator_x_position_ctrl_module_msgs::JointPose");
  QObject::connect(&qnode_,
                   SIGNAL(updateJointPose(manipulator_x_position_ctrl_module_msgs::JointPose)),
                   this,
                   SLOT(updateJointPoseSpinbox(manipulator_x_position_ctrl_module_msgs::JointPose)));

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_set_position_control_mode_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "set_mode";

  qnode_.sendSetModeMsg(msg);
}

void MainWindow::on_go_zero_pose_button_clicked(bool check)
{
  ui_.joint_space_control_checkbox->setChecked(true);

  std_msgs::String msg;
  msg.data = "zero_pose";

  qnode_.sendInitialPoseMsg(msg);
}

void MainWindow::on_go_initial_pose_button_clicked(bool check)
{
  ui_.joint_space_control_checkbox->setChecked(true);

  std_msgs::String msg;
  msg.data = "initial_pose";

  qnode_.sendInitialPoseMsg(msg);
}

void MainWindow::on_get_pre_value_pushbutton_clicked(bool check)
{
  qnode_.getJointPose();
}

void MainWindow::on_send_des_value_pushbutton_clicked(bool check)
{
  manipulator_x_position_ctrl_module_msgs::JointPose msg;

  msg.mov_time = 1.5;
  for (int it=0; it<joint_name_.size(); it++)
  {
    msg.joint_name.push_back(joint_name_[it]);
    msg.position.push_back(((QDoubleSpinBox *) desired_joint_angle_spinbox_[it])->value()*DEGREE2RADIAN);
  }

  qnode_.sendJointPoseMsg(msg);
}

void MainWindow::on_joint_space_control_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.joint_space_control_checkbox->isChecked();

  qnode_.enableJointSpaceControl(msg);
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

void MainWindow::updateJointPoseSpinbox(manipulator_x_position_ctrl_module_msgs::JointPose msg)
{
  for (int name_index=0; name_index<msg.joint_name.size(); name_index++)
  {
    for (int joint_index=0; joint_index<joint_name_.size(); joint_index++)
    {
      if (msg.joint_name[joint_index] == joint_name_[name_index])
      {
        ((QDoubleSpinBox *) present_joint_angle_spinbox_[name_index])->setValue(msg.position[joint_index]*RADIAN2DEGREE);
        ((QDoubleSpinBox *) desired_joint_angle_spinbox_[name_index])->setValue(msg.position[joint_index]*RADIAN2DEGREE);
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

}  // namespace manipulator_x_position_ctrl_gui

