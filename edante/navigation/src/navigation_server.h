#ifndef NAVIGATION_SERVER_H_
#define NAVIGATION_SERVER_H_

#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <navigation/NavigateAction.h>
#include <std_srvs/Empty.h>
#include "motion/GetRobotPosition.h"
#include "motion/MoveTo.h"

#include "definitions.h"

class NavigateAction {
 protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation::NavigateAction> as_;
  std::string action_name_;
  navigation::NavigateFeedback feedback_;
  navigation::NavigateResult result_;

 public:
  NavigateAction(std::string name);

  ~NavigateAction(void);

  void executeCB(const navigation::NavigateGoalConstPtr &goal);

 private:
  motion::GetRobotPosition get_pose_srv_;
  motion::GetRobotPosition start_position_;
  ros::ServiceClient get_pose_client_;
  motion::MoveTo move_to_srv_;
  ros::ServiceClient move_to_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
};


#endif /* NAVIGATION_SERVER_H_ */

