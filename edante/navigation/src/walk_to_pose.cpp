#include "navigation/walk_to_pose.hpp"

WalkToPoseAction::WalkToPoseAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  // Register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&WalkToPoseAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&WalkToPoseAction::preemptCB, this));
  this->rosSetup();
  this->init();
  ROS_INFO("Starting WalkToPose server");
  as_.start();
}

WalkToPoseAction::~WalkToPoseAction(void) {
}

void WalkToPoseAction::init() {
  dist_scalar_ = 3.0f;
  theta_scalar_ = 1.0f;
  dist_thresh_ = 0.05f;
  theta_thresh_ = 0.4f;
}

void WalkToPoseAction::rosSetup() {
  get_robot_pose_client_ = nh_.serviceClient<localization_msgs::GetRobotPose>(
                             "/localization/get_robot_pose", true);
  get_robot_pose_client_.waitForExistence();
  walking_to_pub_ = nh_.advertise <geometry_msgs::Pose2D>
                    ("/world/walking_to", 1, true);
  move_toward_client_ = nh_.serviceClient<motion_msgs::MoveToward>(
                          "/motion/move_toward", true);
  move_toward_client_.waitForExistence();
}

void WalkToPoseAction::goalCB() {
  target_pose_ = as_.acceptNewGoal()->target_pose;
  this->executeCB();
}

void WalkToPoseAction::preemptCB() {
  ROS_INFO("Preempt")
  ;  as_.setPreempted();
}

void WalkToPoseAction::executeCB() {
  bool going = true;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  while (going == true) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }
    // Get absolute robot pose
    get_robot_pose_client_.call(curr_pose_);
    walking_to_pub_.publish(target_pose_);
    ROS_INFO("Curr Pose: %f, %f, %f", curr_pose_.response.pose.x,
             curr_pose_.response.pose.y, curr_pose_.response.pose.theta);
    ROS_INFO("Targ Pose: %f, %f, %f", target_pose_.x, target_pose_.y,
             target_pose_.theta);
    // Calculate vector and angle to target pose
    float rel_target_theta_;
    rel_target_pose_.x = target_pose_.x - curr_pose_.response.pose.x;
    rel_target_pose_.y = target_pose_.y - curr_pose_.response.pose.y;
    if (rel_target_pose_.x != 0.0f) {
      rel_target_theta_ = atan(rel_target_pose_.y / rel_target_pose_.x);
    } else {
      rel_target_theta_ = 0.0f;
    }
    ROS_INFO("Targ Pose: %f, %f", rel_target_pose_.x, rel_target_pose_.y);
    // Calculate relative angle given current orientation
    float rel_distance_error_ = sqrt(pow(rel_target_pose_.x, 2) +
                                     pow(rel_target_pose_.y, 2));
    float rel_theta_error_ = rel_target_theta_ - curr_pose_.response.pose.theta;

    ROS_INFO("Error: Dis=%f, Theta=%f", rel_distance_error_, rel_theta_error_);
    // Reset velocities in-case we have reached a threshold
    move_toward_srv_.request.norm_velocity.x = 0.0f;
    move_toward_srv_.request.norm_velocity.theta = 0.0f;

    float theta_vel = rel_theta_error_ * theta_scalar_;
    if (theta_vel < -1.0) {theta_vel = -1.0;}
    else if (theta_vel > 1.0) {theta_vel = 1.0;}
    move_toward_srv_.request.norm_velocity.theta = theta_vel;
    float x_vel = rel_distance_error_ * dist_scalar_;
    if (x_vel < -1.0) {x_vel = -1.0;}
    else if (x_vel > 1.0) {x_vel = 1.0;}
    move_toward_srv_.request.norm_velocity.x = x_vel;

    ROS_INFO("Vel: x=%f, Theta=%f", x_vel, theta_vel);
    if ((fabs(rel_distance_error_) < dist_thresh_) &&
        (fabs(rel_theta_error_) < theta_thresh_)) {
      ROS_INFO("%s: Arrived", action_name_.c_str());
      result_.outcome = "arrived";
      going = false;
    } else if (fabs(rel_theta_error_) > theta_thresh_) {
      ROS_INFO("%s: Need to Turn", action_name_.c_str());
      result_.outcome = "turn_to_pose";
      going = false;
    }
    move_toward_client_.call(move_toward_srv_);
    ros::spinOnce();
    r.sleep();
  }
  // stop_move_client_.call(stop_move_srv_);

  ROS_INFO("%s: End", action_name_.c_str());
  as_.setSucceeded(result_);
  // move_init_client_.call(move_init_srv_);
}
