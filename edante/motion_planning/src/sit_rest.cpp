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
  as_(nh_, name, boost::bind(&SitRestAction::executeCB, this, _1), false),
  action_name_(name) {
  awake_sub_ = nh_.subscribe("/motion/is_awake", 1, &SitRestAction::awakeCB,
                             this);
  // wake_up_client_ = nh_.serviceClient<std_srvs::Empty>("motion/wake_up", true);
  // get_posture_family_client_ = nh_.serviceClient<motion_msgs::GetPostureFamily>(
  //                                "/motion/get_posture_family", true);
  // get_posture_family_client_.waitForExistence();
  rest_client_ = nh_.serviceClient<std_srvs::Empty>(
                   "/motion/rest", true);
  rest_client_.waitForExistence();
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  stop_move_client_.waitForExistence();
  // set_posture_client_ = nh_.serviceClient<motion_msgs::SetPosture>(
  //                         "/motion/goto_posture", true);
  // set_posture_client_.waitForExistence();
  ROS_INFO("Starting Sit rest action server");
  this->init();
  as_.start();
}

SitRestAction::~SitRestAction(void) {
}

void SitRestAction::init() {
  is_awake_ = true;
  // set_posture_srv_.request.posture_name = "Crouch";
  // set_posture_srv_.request.speed = 0.5f;
}

void SitRestAction::awakeCB(const std_msgs::Bool::ConstPtr& msg) {
  is_awake_ = msg->data;
}

void SitRestAction::executeCB(const motion_planning_msgs::SitRestGoalConstPtr&
                              goal) {
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
  // std::string curr_pos;
  // get_posture_family_client_.call(get_posture_family_srv_);
  // curr_pos = get_posture_family_srv_.response.posture_family;
  // ROS_INFO("Feedback: %s", curr_pos.c_str());

  // If robot is not awake, wake it up with full stiffness
  if (is_awake_ && going) {
    ROS_INFO("Resting");
    stop_move_client_.call(stop_move_srv_);
    rest_client_.call(rest_srv_);
  }

  // If not already crouching, crouch using behaviour
  // if ((curr_pos.compare("Crouch") == 0) && going == true) {
  //   going = false;
  // } else if ( (curr_pos.compare("Crouch") != 0) && going == true) {
  //   set_posture_client_.call(set_posture_srv_);
  //   success = set_posture_srv_.response.success;
  //   going = false;
  // } else { success = false;}

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
