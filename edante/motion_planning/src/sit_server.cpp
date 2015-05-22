/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-22
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Sit down action server
*/

#include "sit_server.h"

SitAction::SitAction(std::string name) :
  as_(nh_, name, boost::bind(&SitAction::executeCB, this, _1), false),
  action_name_(name) {
  wake_up_client_ = nh_.serviceClient<std_srvs::Empty>("motion/wake_up", true);
  get_posture_family_client_ = nh_.serviceClient<motion::GetPostureFamily>(
                                 "/motion/get_posture_family", true);
  get_posture_family_client_.waitForExistence();
  ROS_INFO("Starting Sit down action server");
  as_.start();
}

SitAction::~SitAction(void) {
}

void SitAction::executeCB(const motion_planning::SitGoalConstPtr &goal) {
  bool going = true;
  bool success = true;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  get_posture_family_client_.call(get_posture_family_srv_);

  while (going == true) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }
    get_posture_family_client_.call(get_posture_family_srv_);
    ROS_INFO("Feedback: %s",
             get_posture_family_srv_.response.posture_family.c_str());
    feedback_.feedback = true;
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
