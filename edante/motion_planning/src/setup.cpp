/**
 * @file      setup.cpp
 * @brief     Setup action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/setup.hpp"

SetupAction::SetupAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&SetupAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&SetupAction::preemptCB, this));
  fade_rgb_client_ = nh_.serviceClient<signalling_msgs::FadeRGB>(
                       "/signalling/fade_rgb", true);
  fade_rgb_client_.waitForExistence();
  ROS_INFO("Starting Setup action server");
  this->init();
  as_.start();
}

SetupAction::~SetupAction(void) {
}

void SetupAction::init() {
  initial_rgb_srv_.request.name = "ChestLeds";
  initial_rgb_srv_.request.rgb = Colors::BLACK;
  initial_rgb_srv_.request.duration = 0.0;
  ready_rgb_srv_.request.name = "ChestLeds";
  ready_rgb_srv_.request.rgb = Colors::BLUE;
  ready_rgb_srv_.request.duration = 0.0;
  set_rgb_srv_.request.name = "ChestLeds";
  set_rgb_srv_.request.rgb = Colors::YELLOW;
  set_rgb_srv_.request.duration = 0.0;
  penalized_rgb_srv_.request.name = "ChestLeds";
  penalized_rgb_srv_.request.rgb = Colors::RED;
  penalized_rgb_srv_.request.duration = 0.0;
  playing_rgb_srv_.request.name = "ChestLeds";
  playing_rgb_srv_.request.rgb = Colors::GREEN;
  playing_rgb_srv_.request.duration = 0.0;
  finished_rgb_srv_.request.name = "ChestLeds";
  finished_rgb_srv_.request.rgb = Colors::BLACK;
  finished_rgb_srv_.request.duration = 0.0;
  reset_rgb_srv_.request.name = "ChestLeds";
  reset_rgb_srv_.request.rgb = Colors::BLACK;
  reset_rgb_srv_.request.duration = 0.0;
}

void SetupAction::goalCB() {
  state_ = as_.acceptNewGoal()->state;
  this->executeCB();
}

void SetupAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}


void SetupAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  if (going) {
    if (state_ == GameState::INITIAL) {
      ROS_INFO("INITIAL!");
      fade_rgb_client_.call(initial_rgb_srv_);
    } else if (state_ == GameState::READY) {
      ROS_INFO("READY!");
      fade_rgb_client_.call(reset_rgb_srv_);
      fade_rgb_client_.call(ready_rgb_srv_);
    } else if (state_ == GameState::SET) {
      ROS_INFO("SET!");
      fade_rgb_client_.call(reset_rgb_srv_);
      fade_rgb_client_.call(set_rgb_srv_);
    } else if (state_ == GameState::PENALIZED) {
      ROS_INFO("PENALIZED!");
      fade_rgb_client_.call(reset_rgb_srv_);
      fade_rgb_client_.call(penalized_rgb_srv_);
    } else if (state_ == GameState::PLAYING) {
      ROS_INFO("PLAYING!");
      fade_rgb_client_.call(reset_rgb_srv_);
      fade_rgb_client_.call(playing_rgb_srv_);
    } else if (state_ == GameState::FINISHED) {
      ROS_INFO("FINISHED!");
      fade_rgb_client_.call(finished_rgb_srv_);
    }
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
