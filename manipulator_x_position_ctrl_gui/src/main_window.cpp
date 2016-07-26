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

  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<manipulator_x_position_ctrl_module_msgs::JointPose>("manipulator_x_position_ctrl_module_msgs::JointPose");
  QObject::connect(&qnode_,
                   SIGNAL(updateJointPose(manipulator_x_position_ctrl_module_msgs::JointPose)),
                   this,
                   SLOT(updateJointPoseSpinbox(manipulator_x_position_ctrl_module_msgs::JointPose)));

  qRegisterMetaType<manipulator_x_position_ctrl_module_msgs::KinematicsPose>("manipulator_x_position_ctrl_module_msgs::KinematicsPose");
  QObject::connect(&qnode_,
                   SIGNAL(updateKinematicsPose(manipulator_x_position_ctrl_module_msgs::KinematicsPose)),
                   this,
                   SLOT(updateKinematicsPoseSpinbox(manipulator_x_position_ctrl_module_msgs::KinematicsPose)));

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
  ui_.joint_space_control_checkbox->setChecked(false);
  ui_.task_space_control_checkbox->setChecked(false);
  ui_.motion_planning_checkbox->setChecked(false);

  std_msgs::String msg;
  msg.data = "zero_pose";

  qnode_.sendInitialPoseMsg(msg);
}

void MainWindow::on_go_initial_pose_button_clicked(bool check)
{
  ui_.joint_space_control_checkbox->setChecked(false);
  ui_.task_space_control_checkbox->setChecked(false);
  ui_.motion_planning_checkbox->setChecked(false);

  std_msgs::String msg;
  msg.data = "initial_pose";

  qnode_.sendInitialPoseMsg(msg);
}

void MainWindow::on_get_pre_value_pushbutton_clicked(bool check)
{
  qnode_.getJointPose();
}

void MainWindow::on_get_pre_pose_pushbutton_clicked(bool check)
{
  qnode_.getKinematicsPose();
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

void MainWindow::on_send_des_pose_pushbutton_clicked(bool check)
{
  manipulator_x_position_ctrl_module_msgs::KinematicsPose msg;

  msg.group_name = "arm";
  msg.mov_time = 1.5;

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

void MainWindow::on_joint_space_control_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.joint_space_control_checkbox->isChecked();

  if (msg.data == true)
  {
    ui_.task_space_control_checkbox->setChecked(false);
    ui_.motion_planning_checkbox->setChecked(false);
  }

  qnode_.enableJointSpaceControl(msg);
}

void MainWindow::on_task_space_control_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.task_space_control_checkbox->isChecked();

  if (msg.data == true)
  {
    ui_.joint_space_control_checkbox->setChecked(false);
    ui_.motion_planning_checkbox->setChecked(false);
  }

  qnode_.enableTaskSpaceControl(msg);
}

void MainWindow::on_motion_planning_checkbox_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = ui_.motion_planning_checkbox->isChecked();

  if (msg.data == true)
  {
    ui_.joint_space_control_checkbox->setChecked(false);
    ui_.task_space_control_checkbox->setChecked(false);
  }

  qnode_.enableMotionPlanning(msg);
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

void MainWindow::updateKinematicsPoseSpinbox(manipulator_x_position_ctrl_module_msgs::KinematicsPose msg)
{
  Eigen::MatrixXd position = Eigen::MatrixXd::Zero(3,1);
  position.coeffRef(0,0) = msg.pose.position.x;
  position.coeffRef(1,0) = msg.pose.position.y;
  position.coeffRef(2,0) = msg.pose.position.z;

  for (int index=0; index<present_task_space_position_spinbox_.size(); index++)
  {
    ((QDoubleSpinBox *) present_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
    ((QDoubleSpinBox *) desired_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
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
    ((QDoubleSpinBox *) desired_task_space_orientation_spinbox_[index])->setValue(orientation.coeff(index,0) * RADIAN2DEGREE);
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

