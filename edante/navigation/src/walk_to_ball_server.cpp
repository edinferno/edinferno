#include "navigation/walk_to_ball_server.hpp"

WalkToBallAction::WalkToBallAction(std::string name) :
  as_(nh_, name, boost::bind(&WalkToBallAction::executeCB, this, _1), false),
  action_name_(name) {
  // as_.registerGoalCallback(boost::bind(&WalkToBallAction::goalCB, this));
  ball_pos_sub_ = nh_.subscribe("/vision/ball", 1,
                                &WalkToBallAction::goalCB, this);
  get_pose_client_ = nh_.serviceClient<motion_msgs::GetRobotPosition>(
                       "/motion/get_robot_position", true);
  get_pose_client_.waitForExistence();
  move_toward_client_ = nh_.serviceClient<motion_msgs::MoveToward>(
                          "/motion/move_toward", true);
  move_toward_client_.waitForExistence();
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  stop_move_client_.waitForExistence();
  move_init_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/move_init", true);
  move_init_client_.waitForExistence();
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
}

void WalkToBallAction::goalCB(const vision_msgs::BallDetection::ConstPtr& msg) {
  ball_found_ = msg->is_detected;
  ball_pos_.x = msg->pos_robot.x;
  ball_pos_.y = msg->pos_robot.y;
}

void WalkToBallAction::executeCB(
  const navigation_msgs::WalkToBallGoalConstPtr& goal) {
  bool going = true;
  bool success = false;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  // get_pose_client_.call(start_position_);

  if (!ball_found_) {
    ROS_ERROR("Walk to ball action waiting for ball...");
    while (!ball_found_) {sleep(1);}
  } else {
    ROS_INFO("Ball found! Going towards x: %f, y: %f", ball_pos_.x, ball_pos_.y);
  }

  while (going && ball_found_) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }
    // get_pose_client_.call(get_pose_srv_);

    // FEEDBACK
    // feedback_.curr_pose.x =
    //   get_pose_srv_.response.position.x - start_position_.response.position.x;
    // feedback_.curr_pose.y =
    //   get_pose_srv_.response.position.y - start_position_.response.position.y;
    // feedback_.curr_pose.theta =
    //   get_pose_srv_.response.position.theta - start_position_.response.position.theta;
    // as_.publishFeedback(feedback_);
    // ROS_INFO("Current pos: %f, %f, %f",
    //          feedback_.curr_pose.x,
    //          feedback_.curr_pose.y,
    //          feedback_.curr_pose.theta);

    // Calculate Distance and Orientation error
    target_distance_ = ball_pos_.x;
    if (ball_pos_.x != 0.0f) {
      target_theta_ = atan(ball_pos_.y / ball_pos_.x);
    } else {
      target_theta_ = 0.0f;
    }
    float distance_error = target_distance_; // Relative
    float theta_error = target_theta_;       // Relative
    ROS_INFO("Target dis: %f, Target theta %f", target_distance_, target_theta_);
    // ROS_INFO("Distance_error: %f, Theta error: %f", distance_error, theta_error);

    // Reset velocities in-case we have reached a threshold
    move_toward_srv_.request.norm_velocity.x = 0.0f;
    move_toward_srv_.request.norm_velocity.theta = 0.0f;
    // Rotate first...
    if (fabs(theta_error) > theta_thresh_) {
      if (theta_error < 0) { ROS_INFO("Right"); } else { ROS_INFO("Left"); }
      float theta_vel = theta_error * theta_scalar_;
      if (theta_vel < -1.0) {theta_vel = -1.0;}
      else if (theta_vel > 1.0) {theta_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.theta = theta_vel;
      ROS_INFO("Rot. Speed: %f", move_toward_srv_.request.norm_velocity.theta);
      move_toward_client_.call(move_toward_srv_);
      // ...then straight to ball
    } else if (fabs(distance_error) > dist_thresh_) {
      if (distance_error > 0) { ROS_INFO("Forward"); }
      float forw_vel = distance_error * dist_scalar_;
      if (forw_vel < -1.0) {forw_vel = -1.0;}
      else if (forw_vel > 1.0) {forw_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.x = forw_vel;
      ROS_INFO("Dist. Speed: %f", move_toward_srv_.request.norm_velocity.x);
      move_toward_client_.call(move_toward_srv_);
    }

    // Check if we are close enough to ball
    if ((fabs(distance_error) < dist_thresh_) &&
        (fabs(theta_error) < theta_thresh_)) {
      ROS_INFO("%s: Arrived", action_name_.c_str());
      success = true;
      going = false;
    }
  }

  stop_move_client_.call(stop_move_srv_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    if (!ball_found_) {ROS_INFO("Ball Lost!");}
    as_.setSucceeded(result_);
  }

  move_init_client_.call(move_init_srv_);

}
