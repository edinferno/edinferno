#include "navigation/turn_to_pose.hpp"

TurnToPoseAction::TurnToPoseAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  // Register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&TurnToPoseAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&TurnToPoseAction::preemptCB, this));
  this->rosSetup();
  this->init();
  ROS_INFO("Starting TurnToPose server");
  as_.start();
}

TurnToPoseAction::~TurnToPoseAction(void) {
}

void TurnToPoseAction::init() {
  target_theta_ = 0.0f;
  theta_scalar_ = 1.0f;
  theta_thresh_ = 0.2f;
}

void TurnToPoseAction::rosSetup() {
  ros::service::waitForService("/localization/get_robot_pose");
  get_robot_pose_client_ = nh_.serviceClient<localization_msgs::GetRobotPose>(
                             "/localization/get_robot_pose", true);
  ros::service::waitForService("/motion/move_toward");
  move_toward_client_ = nh_.serviceClient<motion_msgs::MoveToward>(
                          "/motion/move_toward", true);
  move_toward_srv_.request.norm_velocity.x = 0.0f;
}

void TurnToPoseAction::goalCB() {
  target_pose_ = as_.acceptNewGoal()->target_pose;
  this->executeCB();
}

void TurnToPoseAction::preemptCB() {
  ROS_INFO("Preempt")
  ;  as_.setPreempted();
}

void TurnToPoseAction::executeCB() {
  bool going = true;
  bool success = true;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }
    // Get absolute robot pose
    get_robot_pose_client_.call(curr_pose_);

    // Calculate vector and angle to target pose
    rel_target_pose_.x = target_pose_.x - curr_pose_.response.pose.x;
    rel_target_pose_.y = target_pose_.y - curr_pose_.response.pose.y;
    if (rel_target_pose_.x != 0.0f) {
      rel_target_theta_ = atan(rel_target_pose_.y / rel_target_pose_.x);
    } else {
      rel_target_theta_ = 0.0f;
    }
    // Calculate relative angle given current orientation
    float rel_theta_error_ = rel_target_theta_ - curr_pose_.response.pose.theta;

    // Reset velocities in-case we have reached a threshold
    move_toward_srv_.request.norm_velocity.theta = 0.0f;

    float theta_vel = rel_theta_error_ * theta_scalar_;
    if (theta_vel < -1.0) {theta_vel = -1.0;}
    else if (theta_vel > 1.0) {theta_vel = 1.0;}
    move_toward_srv_.request.norm_velocity.theta = theta_vel;
    move_toward_client_.call(move_toward_srv_);

    if (fabs(rel_theta_error_) < theta_thresh_) {
      result_.outcome = "arrived";
      going = false;
    }
    ros::spinOnce();
    r.sleep();
  }
  // stop_move_client_.call(stop_move_srv_);

  ROS_INFO("%s: %s", action_name_.c_str(), result_.outcome.c_str());
  as_.setSucceeded(result_);
  // move_init_client_.call(move_init_srv_);
}
