/**
 * @file      sit_rest.cpp
 * @brief     Sit down action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/sit_rest.hpp"

SitRestAction::SitRestAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&SitRestAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&SitRestAction::preemptCB, this));
  awake_sub_ = nh_.subscribe("/motion/is_awake", 1, &SitRestAction::awakeCB,
                             this);
  ros::service::waitForService("/motion/rest");
  rest_client_ = nh_.serviceClient<std_srvs::Empty>(
                   "/motion/rest", true);
  ros::service::waitForService("/motion/stop_move");
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  ROS_INFO("Starting Sit rest action server");
  this->init();
  as_.start();
}

SitRestAction::~SitRestAction(void) {
}

void SitRestAction::init() {
  is_awake_ = true;
}

void SitRestAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void SitRestAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void SitRestAction::awakeCB(const std_msgs::Bool::ConstPtr& msg) {
  is_awake_ = msg->data;
}

void SitRestAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  if (is_awake_ && going) {
    ROS_INFO("Resting");
    stop_move_client_.call(stop_move_srv_);
    rest_client_.call(rest_srv_);
  }

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setSucceeded(result_);
  }
}
