/**
 * @file      transition.cpp
 * @brief     Transition action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/transition.hpp"

TransitionAction::TransitionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&TransitionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&TransitionAction::preemptCB,
                                          this));
  chest_sub_ = nh_.subscribe("/sensing/chest", 1,
                             &TransitionAction::checkTransition,
                             this);
  ROS_INFO("Starting Transition action server");
  // this->init();
  as_.start();
}

TransitionAction::~TransitionAction(void) {
}

void TransitionAction::init() {
  chest_presses_ = 0;
}

void TransitionAction::goalCB() {
  chest_presses_ = 0;
  state_ = as_.acceptNewGoal()->state;
}

void TransitionAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void TransitionAction::checkTransition(const std_msgs::UInt8::ConstPtr&
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

  if (chest_presses_ == 1) {
    if (state_ == GameState::INITIAL) {
      result_.outcome = "penalized";
      going = false;
    } else if (state_ == GameState::PENALIZED) {
      result_.outcome = "playing";
      going = false;
    } else if (state_ == GameState::PLAYING) {
      result_.outcome = "penalized";
      going = false;
    }
  } else if (chest_presses_ == 2) {
    ROS_INFO("Chest Button twice");
    result_.outcome = state_;
    // going = false;
    // } else if (chest_presses_ == 3) {
    //   ROS_INFO("Chest Button thrice");
    //   going = false;
  }

  if (success) {
    as_.setSucceeded(result_);
  } else {
    result_.outcome = "abort";
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}
