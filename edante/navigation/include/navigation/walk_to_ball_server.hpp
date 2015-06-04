#ifndef WALK_TO_BALL_SERVER_HPP
#define WALK_TO_BALL_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveToward.h>
#include <motion_msgs/GetRobotPosition.h>
#include <navigation_msgs/WalkToBallAction.h>
#include <vision_msgs/BallDetection.h>

class WalkToBallAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::WalkToBallAction> as_;
  std::string action_name_;
  navigation_msgs::WalkToBallFeedback feedback_;
  navigation_msgs::WalkToBallResult result_;

 public:
  WalkToBallAction(std::string name);

  ~WalkToBallAction(void);

  void goalCB(const vision_msgs::BallDetection::ConstPtr& msg);

  void executeCB(const navigation_msgs::WalkToBallGoalConstPtr& goal);

 private:
  float target_distance;
  float target_theta;
  float theta_scalar;
  float dist_scalar;
  float theta_thresh;
  float dist_thresh;
  float the_theta;
  ros::Subscriber ball_pos_sub_;
  // geometry_msgs::Pose2D goal_msg_;

  motion_msgs::GetRobotPosition get_pose_srv_;
  motion_msgs::GetRobotPosition start_position_;
  ros::ServiceClient get_pose_client_;
  motion_msgs::MoveToward move_toward_srv_;
  ros::ServiceClient move_toward_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
};


#endif /* WALK_TO_BALL_SERVER_HPP */

