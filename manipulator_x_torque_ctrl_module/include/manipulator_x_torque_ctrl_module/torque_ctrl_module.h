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
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_framework_common/motion_module.h"

namespace robotis_manipulator_x
{

#define MAX_JOINT_ID  7

class TorqueCtrlJointData
{
public:
  double position_;
  double velocity_;
  double effort_;
};

class TorqueCtrlJointState
{
public:
  TorqueCtrlJointData curr_joint_state_[MAX_JOINT_ID+1];
  TorqueCtrlJointData goal_joint_state_[MAX_JOINT_ID+1];
  double gravity_state_[MAX_JOINT_ID+1];
};


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

  std::map<std::string, int> joint_name_to_id_;

  bool gazebo_;

  KDL::Chain chain_;
  KDL::ChainDynParam *dyn_param_ = NULL;

  void queueThread();
  void setKinematicsChain();
  void calcGravityTerm();

public:
  TorqueCtrlModule();
  virtual ~TorqueCtrlModule();

  /* ROS Topic Callback Functions */
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  /* Parameter */
  TorqueCtrlJointState  *joint_state_;
};

}

#endif /* SRC_ROBOTIS_MANIPULATOR_X_MANIPULATOR_X_TORQUE_CTRL_MODULE_INCLUDE_MANIPULATOR_X_TORQUE_CTRL_MODULE_TORQUE_CTRL_MODULE_H_ */
