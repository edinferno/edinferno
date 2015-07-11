/**
 * @file      stand_up.cpp
 * @brief     Stand up action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/stand_up.hpp"

StandUpAction::StandUpAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&StandUpAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&StandUpAction::preemptCB,
                                          this));
  awake_sub_ = nh_.subscribe("/motion/is_awake", 1, &StandUpAction::awakeCB,
                             this);
  wake_up_client_ = nh_.serviceClient<std_srvs::Empty>("/motion/wake_up", true);
  wake_up_client_.waitForExistence();
  has_fallen_pub_ = nh_.advertise<std_msgs::Bool>("/world/has_fallen", 10);
  get_posture_family_client_ = nh_.serviceClient<motion_msgs::GetPostureFamily>(
                                 "/motion/get_posture_family", true);
  get_posture_family_client_.waitForExistence();
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  stop_move_client_.waitForExistence();
  set_posture_client_ = nh_.serviceClient<motion_msgs::SetPosture>(
                          "/motion/goto_posture", true);
  set_posture_client_.waitForExistence();
  ROS_INFO("Starting Stand up action server");
  this->init();
  as_.start();
}

StandUpAction::~StandUpAction(void) {
}

void StandUpAction::init() {
  is_awake_ = false;
  has_fallen_msg_.data = false;
  set_posture_srv_.request.posture_name = "StandInit";
  set_posture_srv_.request.speed = 1.0f;
}

void StandUpAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void StandUpAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void StandUpAction::awakeCB(const std_msgs::Bool::ConstPtr& msg) {
  is_awake_ = msg->data;
}

void StandUpAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  // Check what posture family we are on (NOT posture)
  std::string curr_pos;
  get_posture_family_client_.call(get_posture_family_srv_);
  curr_pos = get_posture_family_srv_.response.posture_family;
  ROS_INFO("Feedback: %s", curr_pos.c_str());

  // If robot is not awake, wake it up with full stiffness
  if (!is_awake_ && going) {
    ROS_INFO("Stand Up!");
    wake_up_client_.call(wake_up_srv_);
  } else {
    stop_move_client_.call(stop_move_srv_);
  }

  // If not already standing, stand up using behaviour
  if ((curr_pos.compare("Stand") == 0) && going) {
    going = false;
  } else if ( (curr_pos.compare("Stand") != 0) && going) {
    set_posture_client_.call(set_posture_srv_);
    success = set_posture_srv_.response.success;
    going = false;
  } else {success = false;}

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    has_fallen_pub_.publish(has_fallen_msg_);
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setSucceeded(result_);
  }
}
