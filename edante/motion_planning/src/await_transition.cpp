/**
 * @file      await_transition.cpp
 * @brief     AwaitTransition action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/await_transition.hpp"

AwaitTransitionAction::AwaitTransitionAction(ros::NodeHandle nh,
                                             std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&AwaitTransitionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&AwaitTransitionAction::preemptCB,
                                          this));
  chest_sub_ = nh_.subscribe("/sensing/chest", 1,
                             &AwaitTransitionAction::checkTransition,
                             this);
  ROS_INFO("Starting AwaitTransition action server");
  // this->init();
  as_.start();
}

AwaitTransitionAction::~AwaitTransitionAction(void) {
}

void AwaitTransitionAction::init() {
  chest_presses_ = 0;
}

void AwaitTransitionAction::goalCB() {
  chest_presses_ = 0;
  ROS_INFO("New Goal");
  state_ = as_.acceptNewGoal()->state;
  ROS_INFO("State: %i", state_);
}

void AwaitTransitionAction::preemptCB() {
  ROS_INFO("Preempt");
}

void AwaitTransitionAction::checkTransition(const std_msgs::UInt8::ConstPtr&
                                            msg) {
  chest_presses_ = msg->data;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  while (going) {
    ROS_INFO("Checking");
    if (chest_presses_ == 1) {
      ROS_INFO("Chest Button once");
      if (state_ == GameState::INITIAL ||
          state_ == GameState::PENALIZED ||
          state_ == GameState::PLAYING)
      {going = false;}
    } else if (chest_presses_ == 2) {
      ROS_INFO("Chest Button twice");
      going = false;
    } else if (chest_presses_ == 3) {
      ROS_INFO("Chest Button thrice");
      going = false;
    }
    usleep(100000);
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
