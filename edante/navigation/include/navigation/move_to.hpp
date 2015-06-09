#ifndef MOVE_TO_HPP
#define MOVE_TO_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveTo.h>
#include <motion_msgs/GetRobotPosition.h>
#include <navigation_msgs/MoveToAction.h>



class MoveToAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::MoveToAction> as_;
  std::string action_name_;
  navigation_msgs::MoveToFeedback feedback_;
  navigation_msgs::MoveToResult result_;

 public:
  MoveToAction(ros::NodeHandle nh, std::string name);

  ~MoveToAction(void);

  void init();

  void executeCB(const navigation_msgs::MoveToGoalConstPtr& goal);

 private:
  // Variables
  float thresh;

  // ROS
  motion_msgs::GetRobotPosition get_pose_srv_;
  motion_msgs::GetRobotPosition start_position_;
  ros::ServiceClient get_pose_client_;
  motion_msgs::MoveTo move_to_srv_;
  ros::ServiceClient move_to_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
};


#endif /* MOVE_TO_HPP */

