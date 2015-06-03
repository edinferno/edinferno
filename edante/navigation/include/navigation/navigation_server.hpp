#ifndef NAVIGATION_SERVER_HPP
#define NAVIGATION_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveTo.h>
#include <motion_msgs/GetRobotPosition.h>
#include <navigation_msgs/NavigateAction.h>



class NavigateAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::NavigateAction> as_;
  std::string action_name_;
  navigation_msgs::NavigateFeedback feedback_;
  navigation_msgs::NavigateResult result_;

 public:
  NavigateAction(std::string name);

  ~NavigateAction(void);

  void executeCB(const navigation_msgs::NavigateGoalConstPtr& goal);

 private:
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


#endif /* NAVIGATION_SERVER_HPP */

