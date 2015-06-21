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
                             &TransitionAction::checkChestTransition, this);
  game_state_sub_ = nh_.subscribe("/comms/game_state", 1,
                                  &TransitionAction::checkGCTransition, this);
  has_fallen_sub_ = nh_.subscribe("/motion/has_fallen", 1,
                                  &TransitionAction::checkFallenTransition, this);
  penalized_sub_ = nh_.subscribe("/comms/penalized", 1,
                                 &TransitionAction::checkPenalizedTransition, this);
  ROS_INFO("Starting Transition action server");
  as_.start();
}

TransitionAction::~TransitionAction(void) {
}

void TransitionAction::goalCB() {
  chest_presses_ = 0;
  game_state_ = -1;
  state_ = as_.acceptNewGoal()->state;
}

void TransitionAction::preemptCB() {
  as_.setPreempted();
}

void TransitionAction::checkChestTransition(const std_msgs::UInt8::ConstPtr&
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

void TransitionAction::checkGCTransition(const comms_msgs::GameState::ConstPtr&
                                         msg) {
  game_state_ = msg->game_state;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  switch (game_state_) {
  case GameState::INITIAL:
    result_.outcome = "initial";
    break;
  case GameState::READY:
    result_.outcome = "ready";
    break;
  case GameState::SET:
    result_.outcome = "set";
    break;
  case GameState::PLAYING:
    result_.outcome = "playing";
    break;
  case GameState::FINISHED:
    result_.outcome = "finished";
    break;
  case GameState::PENALIZED:
    result_.outcome = "penalized";
    break;
  default:
    success = false;
  }

  if (success) {
    as_.setSucceeded(result_);
  } else {
    result_.outcome = "abort";
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

void TransitionAction::checkFallenTransition(const std_msgs::Bool::ConstPtr&
                                             msg) {
  if (msg->data) {
    ROS_INFO("Fallen! Going to stand up");
    result_.outcome = "stand_up";
    as_.setSucceeded(result_);
  }
}

void TransitionAction::checkPenalizedTransition(const std_msgs::UInt8::ConstPtr&
                                                msg) {
  ROS_INFO("Penalized?");
  if (state_ != GameState::PENALIZED && msg->data >= 1) {
    ROS_INFO("Penalized!");
    result_.outcome = "penalized";
    as_.setSucceeded(result_);
  } else if (state_ == GameState::PENALIZED && msg->data == 0) {
    ROS_INFO("NOT Penalized!");
    result_.outcome = "playing";
    as_.setSucceeded(result_);
  }
}
