/**
 * @file      look_at_ball.cpp
 * @brief     Nao stands up and tracks ball with head
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "navigation/look_at_ball.hpp"

LookAtBallAction::LookAtBallAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  // Register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&LookAtBallAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&LookAtBallAction::preemptCB, this));
  this->rosSetup();
  this->init();
  ROS_INFO("Starting LookAtBall server");
  as_.start();
}

LookAtBallAction::~LookAtBallAction(void) {
}

void LookAtBallAction::init() {
  // Flags
  ball_found_ = false;
  tracking_ = false;
  // Prepare head tracking msgs
  start_head_track_srv_.request.object_type = 0;
  stop_head_track_srv_.request.object_type = 0;
  // Prepare look straight srv
  look_straight_srv_.request.names.push_back("HeadYaw");
  look_straight_srv_.request.names.push_back("HeadPitch");
  look_straight_srv_.request.angle_lists.resize(2);
  look_straight_srv_.request.angle_lists[0].float_list.push_back(0.0f);
  look_straight_srv_.request.angle_lists[1].float_list.push_back(0.0f);
  look_straight_srv_.request.time_lists.resize(2);
  look_straight_srv_.request.time_lists[0].float_list.push_back(0.5f);
  look_straight_srv_.request.time_lists[1].float_list.push_back(0.5f);
  look_straight_srv_.request.is_absolute = true;
}

void LookAtBallAction::rosSetup() {
  ball_pos_sub_ = nh_.subscribe("/vision/ball", 1,
                                &LookAtBallAction::ballCB, this);
  start_head_track_client_ = nh_.serviceClient<vision_msgs::StartHeadTracking>(
                               "/vision/start_head_tracking", true);
  start_head_track_client_.waitForExistence();
  stop_head_track_client_ = nh_.serviceClient<vision_msgs::StopHeadTracking>(
                              "/vision/stop_head_tracking", true);
  stop_head_track_client_.waitForExistence();
  look_client_ = nh_.serviceClient<motion_msgs::AngleInterp>(
                   "/motion/angle_interp", true);
  look_client_.waitForExistence();
}

void LookAtBallAction::ballCB(const vision_msgs::BallDetection::ConstPtr&
                              msg) {
  ball_found_ = msg->is_detected;
}

void LookAtBallAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void LookAtBallAction::preemptCB() {
  ROS_INFO("Preempt")
  ;  as_.setPreempted();
}

void LookAtBallAction::executeCB() {
  bool going = true;
  bool success = true;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  look_client_.call(look_straight_srv_);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }

    if (ball_found_ && !tracking_) {
      // Start tracking ball
      start_head_track_client_.call(start_head_track_srv_);
    } else if (!ball_found_ && tracking_) {
      // Stop tracking ball
      stop_head_track_client_.call(stop_head_track_srv_);
      look_client_.call(look_straight_srv_);
    }
    ros::spinOnce();
    r.sleep();
  }
  // Stop tracking ball
  stop_head_track_client_.call(stop_head_track_srv_);
  look_client_.call(look_straight_srv_);

  // stop_move_client_.call(stop_move_srv_);

  ROS_INFO("%s: %s", action_name_.c_str(), result_.outcome.c_str());
  as_.setSucceeded(result_);
  // move_init_client_.call(move_init_srv_);
}
