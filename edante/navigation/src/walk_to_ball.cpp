#include "navigation/walk_to_ball.hpp"

WalkToBallAction::WalkToBallAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&WalkToBallAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&WalkToBallAction::preemptCB, this));
  ball_pos_sub_ = nh_.subscribe("/vision/ball", 1,
                                &WalkToBallAction::ballCB, this);
  walking_to_pub_ = nh_.advertise <geometry_msgs::Pose2D>
                    ("/world/walking_to", 1, true);
  intention_pub_ = nh_.advertise <std_msgs::Int8>
                   ("/world/intention", 1, true);
  ros::service::waitForService("/motion/get_robot_position");
  get_pose_client_ = nh_.serviceClient<motion_msgs::GetRobotPosition>(
                       "/motion/get_robot_position", true);
  ros::service::waitForService("/motion/move_toward");
  move_toward_client_ = nh_.serviceClient<motion_msgs::MoveToward>(
                          "/motion/move_toward", true);
  ros::service::waitForService("/motion/stop_move");
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  ros::service::waitForService("/motion/move_init");
  move_init_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/move_init", true);
  ros::service::waitForService("/vision/start_head_tracking");
  start_head_track_client_ = nh_.serviceClient<vision_msgs::StartHeadTracking>(
                               "/vision/start_head_tracking", true);
  ros::service::waitForService("/vision/stop_head_tracking");
  stop_head_track_client_ = nh_.serviceClient<vision_msgs::StopHeadTracking>(
                              "/vision/stop_head_tracking", true);
  ros::service::waitForService("/motion_planning/monitor_mode");
  monitor_client_ = nh_.serviceClient<motion_planning_msgs::MonitorMode>(
                      "/motion_planning/monitor_mode", true);
  start_position_.request.use_sensors = true;
  get_pose_srv_.request.use_sensors = true;
  this->init();
  ROS_INFO("Starting WalkToBall server");
  as_.start();
}

WalkToBallAction::~WalkToBallAction(void) {
}

void WalkToBallAction::init() {
  ball_found_ = false;
  target_distance_ = 0.0f;
  target_theta_ = 0.0f;
  dist_scalar_ = 3.0f;
  theta_scalar_ = 1.0f;
  dist_thresh_ = 0.05f;
  theta_thresh_ = 0.2f;
  ball_lost_monitor_srv_.request.monitor_mode = MonitorMode::BALL_LOST;
}

void WalkToBallAction::ballCB(const vision_msgs::BallDetection::ConstPtr& msg) {
  ball_found_ = msg->is_detected;
  ball_pos_.x = msg->pos_robot.x;
  ball_pos_.y = msg->pos_robot.y;
}

void WalkToBallAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void WalkToBallAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void WalkToBallAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  // Start tracking ball
  move_init_client_.call(move_init_srv_);
  start_head_track_srv_.request.object_type = 0;
  start_head_track_client_.call(start_head_track_srv_);

  while (going && ball_found_) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    std_msgs::Int8 play_ball;
    play_ball.data = 3;
    intention_pub_.publish(play_ball);
    // walking_to_pub_.publish(absolute_ball_pos_); // Waiting for Svet
    // Calculate Distance and Orientation error
    target_distance_ = ball_pos_.x;
    if (ball_pos_.x != 0.0f) {
      target_theta_ = atan(ball_pos_.y / ball_pos_.x);
    } else {
      target_theta_ = 0.0f;
    }
    float distance_error = target_distance_; // Relative
    float theta_error = target_theta_;       // Relative

    // Reset velocities in-case we have reached a threshold
    move_toward_srv_.request.norm_velocity.x = 0.0f;
    move_toward_srv_.request.norm_velocity.theta = 0.0f;
    // Rotate first...
    float theta_vel = theta_error * theta_scalar_;
    if (theta_vel < -1.0) {theta_vel = -1.0;}
    else if (theta_vel > 1.0) {theta_vel = 1.0;}
    move_toward_srv_.request.norm_velocity.theta = theta_vel;
    // ...then straight to ball
    if (fabs(distance_error) > dist_thresh_) {
      float forw_vel = distance_error * dist_scalar_;
      if (forw_vel < -1.0) {forw_vel = -1.0;}
      else if (forw_vel > 1.0) {forw_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.x = forw_vel;
    }
    move_toward_client_.call(move_toward_srv_);

    // Check if we are close enough to ball
    if ((fabs(distance_error) < dist_thresh_) &&
        (fabs(theta_error) < theta_thresh_)) {
      ROS_INFO("%s: Arrived", action_name_.c_str());
      success = true;
      going = false;
    }
    ros::spinOnce();
    r.sleep();
  }
  // Stop tracking ball
  stop_head_track_srv_.request.object_type = 0;
  stop_head_track_client_.call(stop_head_track_srv_);

  stop_move_client_.call(stop_move_srv_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    if (!ball_found_) {
      monitor_client_.call(ball_lost_monitor_srv_);
      ROS_INFO("Ball Lost!");
    }
    as_.setAborted(result_);
  }

  move_init_client_.call(move_init_srv_);

}
