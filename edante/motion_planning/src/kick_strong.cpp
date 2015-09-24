/**
 * @file      kick_strong.cpp
 * @brief     Strong kicking action with 4.2 second execution
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/kick_strong.hpp"

KickStrongAction::KickStrongAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&KickStrongAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&KickStrongAction::preemptCB,
                                          this));
  ros::service::waitForService("/motion/move_init");
  move_init_client_ =
    nh_.serviceClient<std_srvs::Empty>("/motion/move_init", true);
  ros::service::waitForService("/motion/enable_balance");
  balance_client_ =
    nh_.serviceClient<motion_msgs::Enable>("/motion/enable_balance", true);
  ros::service::waitForService("/motion/position_interpolation");
  pos_interp_client_ =
    nh_.serviceClient<motion_msgs::PositionInterpolation>("/motion/position_interpolation",
                                                          true);
  ros::service::waitForService("/motion/foot_state");
  foot_client_ =
    nh_.serviceClient<motion_msgs::FootState>("/motion/foot_state", true);
  ros::service::waitForService("/motion/goto_balance");
  goto_balance_client_ =
    nh_.serviceClient<motion_msgs::GoToBalance>("/motion/goto_balance", true);
  ROS_INFO("Starting KickStrong action server");
  this->init();
  as_.start();
}

KickStrongAction::~KickStrongAction(void) {
}

void KickStrongAction::init() {
  // KICK PARAMETERS
  // Times
  balance_init_ = 1.0f;
  kick_time_ = 0.2f;
  move_time_ = 0.4f;
  lift_time_ = 0.5f;
  balance_end_ = 0.8f;
  // Distance
  leg_lift_ = 0.05f;
  foot_retraction_ = 0.1f;
  foot_forward_kick_ = 0.1f;
  foot_lift_ = 0.015f;
  foot_rot_kick_ = 0.6f;

  // ROS SERVICES
  enable_balance_.request.is_enabled = true;
  disable_balance_.request.is_enabled = false;
  left_foot_state_.request.state_name = "Fixed";
  right_foot_state_.request.state_name = "Fixed";
  left_foot_state_.request.support_leg = "LLeg";
  right_foot_state_.request.support_leg = "RLeg";
  goto_balance_init_.request.duration = balance_init_;
  goto_balance_final_.request.duration = balance_end_;
  goto_balance_final_.request.support_leg = "Legs";
  this->setupLiftLeg();
  this->setupFootBack();
  this->setupKickForw();
  this->setupRetractFoot();
  this->setupLowerLeg();
}

void KickStrongAction::goalCB() {
  kick_type_ = as_.acceptNewGoal()->kick_type;
  this->executeCB();
}

void KickStrongAction::preemptCB() {
  going = false;
  as_.setPreempted();
}

void KickStrongAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
  }

  // Select kick type
  this->setKickType();
  // Enable balancing
  balance_client_.call(enable_balance_);
  // Prepare foot states for balancing
  foot_client_.call(left_foot_state_);
  foot_client_.call(right_foot_state_);
  // Shift weight to support leg
  goto_balance_client_.call(goto_balance_init_);
  // Disable balancing
  balance_client_.call(disable_balance_);
  // Lift kicking leg
  pos_interp_client_.call(lift_leg_);
  // Move kicking foot back
  pos_interp_client_.call(foot_back_);
  // Kick forwards
  pos_interp_client_.call(kick_forw_);
  // Retract kicking leg
  pos_interp_client_.call(retract_foot_);
  // Put kicking foot down
  pos_interp_client_.call(lower_leg_);
  // Re-enable balancing
  balance_client_.call(enable_balance_);
  // Re-set foot states for balancing (May not be required)
  foot_client_.call(left_foot_state_);
  foot_client_.call(right_foot_state_);
  // Shift weight to both legs
  goto_balance_client_.call(goto_balance_final_);
  // Disable balancing
  balance_client_.call(disable_balance_);

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

void KickStrongAction::setKickType() {
  std::string leg;
  std::string support_leg;
  if (kick_type_ == KickType::LEFT) {
    // Left kick
    leg = "LLeg";
    support_leg = "RLeg";
  } else if (kick_type_ == KickType::RIGHT) {
    // Right kick
    leg = "RLeg";
    support_leg = "LLeg";
  } else {/* ERROR, UNKNOWN KICK */}
  goto_balance_init_.request.support_leg = support_leg;
  lift_leg_.request.chain_name = leg;
  foot_back_.request.chain_name = leg;
  kick_forw_.request.chain_name = leg;
  retract_foot_.request.chain_name = leg;
  lower_leg_.request.chain_name = leg;
}

void KickStrongAction::setupLiftLeg() {
  lift_leg_.request.space = FRAME_ROBOT;
  lift_leg_.request.path.traj_points.resize(1);
  lift_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg_.request.path.traj_points[0].float_list.push_back(leg_lift_);
  lift_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg_.request.axis_mask = 63;
  lift_leg_.request.durations.resize(1);
  lift_leg_.request.durations[0] = lift_time_;
  lift_leg_.request.is_absolute = false;
}

void KickStrongAction::setupFootBack() {
  foot_back_.request.space = FRAME_ROBOT;
  foot_back_.request.path.traj_points.resize(1);
  foot_back_.request.path.traj_points[0].float_list.push_back(-foot_retraction_);
  foot_back_.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back_.request.path.traj_points[0].float_list.push_back(leg_lift_);
  foot_back_.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back_.request.path.traj_points[0].float_list.push_back(foot_rot_kick_);
  foot_back_.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back_.request.axis_mask = 63;
  foot_back_.request.durations.resize(1);
  foot_back_.request.durations[0] = move_time_;
  foot_back_.request.is_absolute = false;
}

void KickStrongAction::setupKickForw() {
  kick_forw_.request.space = FRAME_ROBOT;
  kick_forw_.request.path.traj_points.resize(1);
  kick_forw_.request.path.traj_points[0].float_list.push_back(
    foot_retraction_ + foot_forward_kick_);
  kick_forw_.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw_.request.path.traj_points[0].float_list.push_back(foot_lift_);
  kick_forw_.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw_.request.path.traj_points[0].float_list.push_back(-foot_rot_kick_);
  kick_forw_.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw_.request.axis_mask = 63;
  kick_forw_.request.durations.resize(1);
  kick_forw_.request.durations[0] = kick_time_;
  kick_forw_.request.is_absolute = false;
}

void KickStrongAction::setupRetractFoot() {
  retract_foot_.request.space = FRAME_ROBOT;
  retract_foot_.request.path.traj_points.resize(1);
  retract_foot_.request.path.traj_points[0].float_list.push_back(
    -foot_forward_kick_);
  retract_foot_.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_foot_.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_foot_.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_foot_.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_foot_.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_foot_.request.axis_mask = 63;
  retract_foot_.request.durations.resize(1);
  retract_foot_.request.durations[0] = move_time_;
  retract_foot_.request.is_absolute = false;
}

void KickStrongAction::setupLowerLeg() {
  lower_leg_.request.space = FRAME_ROBOT;
  lower_leg_.request.path.traj_points.resize(1);
  lower_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_leg_.request.path.traj_points[0].float_list.push_back(-foot_lift_);
  lower_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_leg_.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_leg_.request.axis_mask = 63;
  lower_leg_.request.durations.resize(1);
  lower_leg_.request.durations[0] = move_time_;
  lower_leg_.request.is_absolute = false;
}
