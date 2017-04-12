/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/manipulator_x_position_ctrl_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_x_position_ctrl_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::sendSetModeMsg(std_msgs::String msg)
{
  std_msgs::String str_msg;

  str_msg.data = "position_ctrl_module";
  set_ctrl_module_pub_.publish(str_msg);

  set_position_ctrl_mode_msg_pub_.publish(msg);
}

void QNode::sendInitialPoseMsg(std_msgs::String msg)
{
  set_initial_pose_msg_pub_.publish(msg);
}

void QNode::sendJointPoseMsg(manipulator_x_position_ctrl_module_msgs::JointPose msg)
{
  set_joint_pose_msg_pub_.publish(msg);
}

void QNode::enableJointSpaceControl(std_msgs::Bool msg)
{
  enable_joint_space_control_pub_.publish(msg);
}

void QNode::getJointPose()
{
  manipulator_x_position_ctrl_module_msgs::GetJointPose srv;

  if (get_joint_pose_client_.call(srv))
  {
    manipulator_x_position_ctrl_module_msgs::JointPose msg;

    msg.joint_name = srv.response.position_ctrl_joint_pose.joint_name;
    msg.position = srv.response.position_ctrl_joint_pose.position;

    Q_EMIT updateJointPose(msg);
  }
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"manipulator_x_position_ctrl_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_ctrl_module_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  set_position_ctrl_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/position_ctrl/set_mode_msg", 0);
  set_initial_pose_msg_pub_ = n.advertise<std_msgs::String>("/robotis/position_ctrl/set_initial_pose_msg", 0);
  set_joint_pose_msg_pub_ = n.advertise<manipulator_x_position_ctrl_module_msgs::JointPose>("/robotis/position_ctrl/set_joint_pose_msg", 0);

  enable_joint_space_control_pub_ = n.advertise<std_msgs::Bool>("/robotis/position_ctrl/enable_joint_space_control_msg", 0);

  get_joint_pose_client_ = n.serviceClient<manipulator_x_position_ctrl_module_msgs::GetJointPose>("/robotis/position_ctrl/get_joint_pose", 0);

  status_msg_sub_ = n.subscribe("/robotis/status", 10, &QNode::statusMsgCallback, this);

  start();
  return true;
}

void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNode::run()
{
  ros::Rate loop_rate(125);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;

    std::stringstream _sender;
    _sender << "[" << sender << "] ";

    switch ( level ) {
        case(Debug) : {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
            break;
        }
        case(Info) : {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
            break;
        }
        case(Warn) : {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Error) : {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
        case(Fatal) : {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace manipulator_x_position_ctrl_gui
