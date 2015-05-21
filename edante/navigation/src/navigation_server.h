#ifndef NAVIGATION_SERVER_H_
#define NAVIGATION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <navigation/NavigateAction.h>

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

};


#endif /* NAVIGATION_SERVER_H_ */

