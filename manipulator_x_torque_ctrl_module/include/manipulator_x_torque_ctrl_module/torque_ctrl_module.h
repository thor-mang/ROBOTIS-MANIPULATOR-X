/*
 * torque_ctrl_module.h
 *
 *  Created on: Jul 6, 2016
 *      Author: sch
 */

#ifndef SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_TORQUE_CTRL_MODULE_INCLUDE_MANIPULATOR_X_TORQUE_CTRL_MODULE_TORQUE_CTRL_MODULE_H_
#define SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_TORQUE_CTRL_MODULE_INCLUDE_MANIPULATOR_X_TORQUE_CTRL_MODULE_TORQUE_CTRL_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "manipulator_x_torque_ctrl_module_msgs/JointGain.h"
#include "manipulator_x_torque_ctrl_module_msgs/GetJointGain.h"

#include "manipulator_x_torque_ctrl_module_msgs/GetJointGain.h"
#include "manipulator_x_torque_ctrl_module_msgs/GetJointPose.h"

namespace robotis_manipulator_x
{

#define MAX_JOINT_ID    6
#define TASK_DEMENSION  6
#define ITERATION_TIME  0.008

class TorqueCtrlModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<TorqueCtrlModule>
{
private:
  int control_cycle_msec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Publisher  status_msg_pub_;
  ros::Publisher  set_ctrl_module_pub_;

  ros::ServiceServer get_joint_gain_server_;
  ros::ServiceServer get_joint_pose_server_;

  std::map<std::string, int> joint_name_to_id_;

  bool gazebo_;
  bool gripper_;

  bool joint_control_mode_;
  bool force_control_mode_;

  KDL::Chain chain_;
  KDL::ChainDynParam *dyn_param_ = NULL;
  boost::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;

  std::string gain_path_;

  Eigen::VectorXd p_gain_;
  Eigen::VectorXd i_gain_;
  Eigen::VectorXd d_gain_;

  Eigen::VectorXd error_;
  Eigen::VectorXd error_prior_;
  Eigen::VectorXd integral_;
  Eigen::VectorXd derivative_;

  Eigen::VectorXd gravity_term_;
  Eigen::VectorXd coriolis_term_;
  Eigen::MatrixXd mass_term_;

  Eigen::MatrixXd jacobian_;

  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd present_joint_effort_;

  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd goal_joint_velocity_;
  Eigen::VectorXd goal_joint_acceleration_;
  Eigen::VectorXd goal_joint_effort_;

  Eigen::VectorXd goal_task_wrench_;

  void queueThread();

  void setKinematicsChain();
  void calcGravityTerm();
  void calcCoriolisTerm();
  void calcMassTerm();
  void calcJacobian();

  void parseGainData(const std::string &path);
  void saveGainData(const std::string &path);

public:
  TorqueCtrlModule();
  virtual ~TorqueCtrlModule();

  /* ROS Topic Callback Functions */
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);
  void saveGainMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setGainMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointGain::ConstPtr& msg);
  void setJointPoseMsgCallback(const manipulator_x_torque_ctrl_module_msgs::JointPose::ConstPtr& msg);
  void setWrenchMsgCallback(const geometry_msgs::Wrench::ConstPtr& msg);

  void enableJointControlMsgCallback(const std_msgs::Bool::ConstPtr& msg);
  void enableForceControlMsgCallback(const std_msgs::Bool::ConstPtr& msg);

  bool getJointGainCallback(manipulator_x_torque_ctrl_module_msgs::GetJointGain::Request &req,
                            manipulator_x_torque_ctrl_module_msgs::GetJointGain::Response &res);

  bool getJointPoseCallback(manipulator_x_torque_ctrl_module_msgs::GetJointPose::Request &req,
                            manipulator_x_torque_ctrl_module_msgs::GetJointPose::Response &res);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};

}

#endif /* SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_TORQUE_CTRL_MODULE_INCLUDE_MANIPULATOR_X_TORQUE_CTRL_MODULE_TORQUE_CTRL_MODULE_H_ */
