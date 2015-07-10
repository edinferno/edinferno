#include "navigation/align_to_pose.hpp"

AlignToPoseAction::AlignToPoseAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  // Register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&AlignToPoseAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&AlignToPoseAction::preemptCB, this));
  this->rosSetup();
  this->init();
  ROS_INFO("Starting AlignToPose server");
  as_.start();
}

AlignToPoseAction::~AlignToPoseAction(void) {
}

void AlignToPoseAction::init() {
  x_scalar_ = 1.0f;
  y_scalar_ = 1.0f;
  dist_thresh_ = 0.10f;
  theta_scalar_ = 1.0f;
  theta_thresh_ = 0.2f;
}

void AlignToPoseAction::rosSetup() {
  get_robot_pose_client_ = nh_.serviceClient<localization_msgs::GetRobotPose>(
                             "/localization/get_robot_pose", true);
  get_robot_pose_client_.waitForExistence();
  move_toward_client_ = nh_.serviceClient<motion_msgs::MoveToward>(
                          "/motion/move_toward", true);
  move_toward_client_.waitForExistence();
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  stop_move_client_.waitForExistence();
  move_init_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/move_init", true);
  move_init_client_.waitForExistence();
}

void AlignToPoseAction::goalCB() {
  target_pose_ = as_.acceptNewGoal()->target_pose;
  this->executeCB();
}

void AlignToPoseAction::preemptCB() {
  ROS_INFO("Preempt")
  ;  as_.setPreempted();
}

void AlignToPoseAction::executeCB() {
  bool going = true;
  bool success = true;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  while (going == true) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }
    // Get absolute robot pose
    get_robot_pose_client_.call(curr_pose_);

    // Calculate relative distance and angle to target pose
    rel_error_.x = target_pose_.x - curr_pose_.response.pose.x;
    rel_error_.y = target_pose_.y - curr_pose_.response.pose.y;
    rel_error_.theta = target_pose_.theta - curr_pose_.response.pose.theta;

    // Reset velocities in-case we have reached a threshold
    move_toward_srv_.request.norm_velocity.x = 0.0f;
    move_toward_srv_.request.norm_velocity.y = 0.0f;
    move_toward_srv_.request.norm_velocity.theta = 0.0f;

    float theta_vel = rel_error_.theta * theta_scalar_;
    if (theta_vel < -1.0) {theta_vel = -1.0;}
    else if (theta_vel > 1.0) {theta_vel = 1.0;}
    move_toward_srv_.request.norm_velocity.theta = theta_vel;
    if (rel_error_.theta < theta_thresh_ * 2) {
      float x_vel = rel_error_.x * x_scalar_;
      if (x_vel < -1.0) {x_vel = -1.0;}
      else if (x_vel > 1.0) {x_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.x = x_vel;
      float y_vel = rel_error_.y * y_scalar_;
      if (y_vel < -1.0) {y_vel = -1.0;}
      else if (y_vel > 1.0) {y_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.y = y_vel;
    }
    ROS_INFO("Target_pose: %f, %f, %f",
             target_pose_.x, target_pose_.y, target_pose_.theta);
    ROS_INFO("curr_pose: %f, %f, %f",
             curr_pose_.response.pose.x, curr_pose_.response.pose.y,
             curr_pose_.response.pose.theta);
    ROS_INFO("Rel_error: %f, %f, %f",
             rel_error_.x, rel_error_.y, rel_error_.theta);
    if ((fabs(rel_error_.x) < dist_thresh_) &&
        (fabs(rel_error_.y) < dist_thresh_) &&
        (fabs(rel_error_.theta) < theta_thresh_)) {
      ROS_INFO("%s: Arrived", action_name_.c_str());
      success = true;
      going = false;
    }
    move_toward_client_.call(move_toward_srv_);
    ros::spinOnce();
    r.sleep();
  }
  stop_move_client_.call(stop_move_srv_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setSucceeded(result_);
  }
  move_init_client_.call(move_init_srv_);
}
