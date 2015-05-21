#include "navigation_server.h"

NavigateAction::NavigateAction(std::string name) :
  as_(nh_, name, boost::bind(&NavigateAction::executeCB, this, _1), false),
  action_name_(name) {
  ROS_INFO("Starting Navigation server");
  as_.start();
}

NavigateAction::~NavigateAction(void) {
}

void NavigateAction::executeCB(const navigation::NavigateGoalConstPtr &goal) {
  bool success = true;
  ROS_INFO("Executing goal");

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
  }

  feedback_.curr_pose = goal->target_pose;
  as_.publishFeedback(feedback_);
  ROS_INFO("x:%f, y:%f, theta:%f",
           feedback_.curr_pose.x,
           feedback_.curr_pose.y,
           feedback_.curr_pose.theta);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
}
