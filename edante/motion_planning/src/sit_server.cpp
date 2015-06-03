/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-22
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Sit down action server
*/

#include "motion_planning/sit_server.hpp"

SitAction::SitAction(std::string name) :
  as_(nh_, name, boost::bind(&SitAction::executeCB, this, _1), false),
  action_name_(name) {
  wake_up_client_ = nh_.serviceClient<std_srvs::Empty>("motion/wake_up", true);
  get_posture_family_client_ = nh_.serviceClient<motion_msgs::GetPostureFamily>(
                                 "/motion/get_posture_family", true);
  get_posture_family_client_.waitForExistence();
  stopMoveClient = nh_.serviceClient<std_srvs::Empty>(
                     "/motion/stop_move", true);
  stopMoveClient.waitForExistence();
  set_posture_client_ = nh_.serviceClient<motion_msgs::SetPosture>(
                          "/motion/goto_posture", true);
  set_posture_client_.waitForExistence();
  set_posture_srv_.request.posture_name = "Crouch";
  set_posture_srv_.request.speed = 0.5f;
  ROS_INFO("Starting Sit up action server");
  as_.start();
}

SitAction::~SitAction(void) {
}

void SitAction::executeCB(const motion_planning_msgs::SitGoalConstPtr& goal) {
  bool going = true;
  bool success = true;
  std::string curr_pos;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  get_posture_family_client_.call(get_posture_family_srv_);
  curr_pos = get_posture_family_srv_.response.posture_family;
  ROS_INFO("Feedback: %s", curr_pos.c_str());

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  if ((curr_pos.compare("Crouch") == 0) && going == true) {
    ROS_INFO("Already Crouching!");
    get_posture_family_client_.call(get_posture_family_srv_);
    curr_pos = get_posture_family_srv_.response.posture_family;
    ROS_INFO("Feedback: %s", curr_pos.c_str());
    going = false;
  } else if ( (curr_pos.compare("Crouch") != 0) && going == true) {
    ROS_INFO("Initiate sit down from %s", curr_pos.c_str());
    set_posture_client_.call(set_posture_srv_);
    success = true;
    going = false;
  } else { success = false;}

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
