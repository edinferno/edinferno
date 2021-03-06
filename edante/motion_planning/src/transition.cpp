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
  has_fallen_sub_ = nh_.subscribe("/world/has_fallen", 1,
                                  &TransitionAction::checkFallenTransition, this);
  penalized_sub_ = nh_.subscribe("/comms/penalized", 1,
                                 &TransitionAction::checkPenalizedTransition, this);
  smach_online_sub_ = nh_.subscribe("/smach/online", 1,
                                    &TransitionAction::checkSmachOnline, this);
  manual_penalized_pub_ =
    nh_.advertise<std_msgs::UInt8>("/world/manually_penalized", 1, true);
  ros::service::waitForService("/motion/rest");
  rest_client_ = nh_.serviceClient<std_srvs::Empty>(
                   "/motion/rest", true);
  ros::service::waitForService("/motion/wake_up");
  wakeup_client_ = nh_.serviceClient<std_srvs::Empty>(
                     "/motion/wake_up", true);
  ros::service::waitForService("/motion/goto_posture");
  stand_client_ = nh_.serviceClient<motion_msgs::SetPosture>(
                    "/motion/goto_posture", true);
  ROS_INFO("Starting Transition action server");
  this->init();
  as_.start();
}

TransitionAction::~TransitionAction(void) {
}

void TransitionAction::goalCB() {
  chest_presses_ = 0;
  state_ = as_.acceptNewGoal()->state;
}

void TransitionAction::preemptCB() {
  as_.setPreempted();
}

void TransitionAction::init() {
  disabled_ = false;
  smach_online_ = false;
  stand_srv_.request.posture_name = "Stand";
  stand_srv_.request.speed = 1.0f;
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

  std_msgs::UInt8 man_pen_msg_;
  if (chest_presses_ == 1) {
    if (state_ == GameState::INITIAL) {
      result_.outcome = "penalized";
      man_pen_msg_.data = GAMECONTROLLER_RETURN_MSG_MAN_PENALISE;
      manual_penalized_pub_.publish(man_pen_msg_);
      going = false;
    } else if (state_ == GameState::PENALIZED) {
      result_.outcome = "playing";
      man_pen_msg_.data = GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;
      manual_penalized_pub_.publish(man_pen_msg_);
      going = false;
    } else if (state_ == GameState::PLAYING) {
      result_.outcome = "penalized";
      man_pen_msg_.data = GAMECONTROLLER_RETURN_MSG_MAN_PENALISE;
      manual_penalized_pub_.publish(man_pen_msg_);
      going = false;
    }
  } else if (chest_presses_ == 2) {
    result_.outcome = state_;
    if (!disabled_) {
      rest_client_.call(rest_srv_);
      disabled_ = true;
    } else if (disabled_) {
      wakeup_client_.call(wakeup_srv_);
      stand_client_.call(stand_srv_);
      disabled_ = false;
    }
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

void TransitionAction::checkGCTransition(const std_msgs::UInt8::ConstPtr&
                                         msg) {
  if (smach_online_) {
    game_state_ = msg->data;
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
      as_.setSucceeded(result_);
      break;
    case GameState::READY:
      result_.outcome = "ready";
      as_.setSucceeded(result_);
      break;
    case GameState::SET:
      result_.outcome = "set";
      as_.setSucceeded(result_);
      break;
    case GameState::PLAYING:
      result_.outcome = "playing";
      as_.setSucceeded(result_);
      break;
    case GameState::FINISHED:
      result_.outcome = "finished";
      as_.setSucceeded(result_);
      break;
    default:
      success = false;
    }

    if (!success) {
      result_.outcome = "abort";
      ROS_INFO("%s: Failed!", action_name_.c_str());
      as_.setAborted(result_);
    }
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
  if (smach_online_) {
    if (state_ != GameState::PENALIZED && msg->data != PENALTY_NONE) {
      result_.outcome = "penalized";
      as_.setSucceeded(result_);
    } else if (state_ == GameState::PENALIZED && msg->data == PENALTY_NONE) {
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
      default:
        ROS_INFO("ERROR, got %d game_state", game_state_);
      }
      as_.setSucceeded(result_);
    }
  }
}

void TransitionAction::checkSmachOnline(const std_msgs::Bool::ConstPtr&
                                        msg) {
  smach_online_ = msg->data;
}
