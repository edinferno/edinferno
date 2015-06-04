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
  ROS_INFO("Starting WalkToBall server");
  target_distance = 0.0f;
  target_theta = 0.0f;
  theta_scalar = 1.0f;
  dist_scalar = 3.0f;
  theta_thresh = 0.2f;
  dist_thresh = 0.05f;
  as_.start();
}

WalkToBallAction::~WalkToBallAction(void) {
}

void WalkToBallAction::goalCB(const vision_msgs::BallDetection::ConstPtr& msg) {
  ROS_INFO("New Goal Received");
  target_distance = msg->pos_robot.x;
  target_theta = msg->pos_robot.z;
}

void WalkToBallAction::executeCB(
  const navigation_msgs::WalkToBallGoalConstPtr& goal) {
  bool going = true;
  bool success = true;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  get_pose_client_.call(start_position_);

  if (target_distance == 0.0f && target_theta == 0.0f) {
    target_distance = goal->ball_pose.x;
    target_theta = goal->ball_pose.theta;
    ROS_ERROR("Ball position not available, using agent goal %f, %f",
              target_distance, target_theta);
  }

  // ROTATE FIRST, THEN STRAIGHT TO GOAL
  // move_toward_srv_.request.control_points.resize(2);
  // move_toward_srv_.request.control_points[0].x = 0.0f;
  // move_toward_srv_.request.control_points[0].y = 0.0f;
  // move_toward_srv_.request.control_points[0].theta = goal->target_pose.theta;
  // move_toward_srv_.request.control_points[1].x = goal->target_pose.x;
  // move_toward_srv_.request.control_points[1].y = 0.0f;
  // move_toward_srv_.request.control_points[1].theta = 0.0f;
  // move_toward_client_.call(move_toward_srv_);
  while (going == true) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }
    get_pose_client_.call(get_pose_srv_);

    feedback_.curr_pose.x =
      get_pose_srv_.response.position.x - start_position_.response.position.x;
    feedback_.curr_pose.y =
      get_pose_srv_.response.position.y - start_position_.response.position.y;
    feedback_.curr_pose.theta =
      get_pose_srv_.response.position.theta - start_position_.response.position.theta;
    as_.publishFeedback(feedback_);

    ROS_INFO("Current pos: %f, %f, %f",
             feedback_.curr_pose.x,
             feedback_.curr_pose.y,
             feedback_.curr_pose.theta);
    ROS_INFO("Target dis: %f, Target theta %f",
             target_distance,
             target_theta);
    // float distance = sqrt(pow(feedback_.curr_pose.x, 2) +
    //                       pow(feedback_.curr_pose.y, 2) );
    // float distance_error = target_distance - distance;            // Absolute
    // float theta_error = target_theta - feedback_.curr_pose.theta; // Absolute
    float distance_error = target_distance; // Relative
    float theta_error = target_theta;       // Relative
    ROS_INFO("Distance_error: %f, Theta error: %f", distance_error, theta_error);

    // ROTATE FIRST, THEN STRAIGHT TO GOAL
    move_toward_srv_.request.norm_velocity.x = 0.0f;
    move_toward_srv_.request.norm_velocity.theta = 0.0f;
    if (fabs(theta_error) > theta_thresh) {
      if (theta_error < 0) { ROS_INFO("Right"); } else { ROS_INFO("Left"); }
      float theta_vel = theta_error * theta_scalar;
      if (theta_vel < -1.0) {theta_vel = -1.0;}
      else if (theta_vel > 1.0) {theta_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.theta = theta_vel;
      ROS_INFO("Rot. Speed: %f", move_toward_srv_.request.norm_velocity.theta);
      move_toward_client_.call(move_toward_srv_);

    } else if (fabs(distance_error) > dist_thresh) {
      if (distance_error > 0) { ROS_INFO("Forward"); }
      float forw_vel = distance_error * dist_scalar;
      if (forw_vel < -1.0) {forw_vel = -1.0;}
      else if (forw_vel > 1.0) {forw_vel = 1.0;}
      move_toward_srv_.request.norm_velocity.x = forw_vel;
      ROS_INFO("Dist. Speed: %f", move_toward_srv_.request.norm_velocity.x);
      move_toward_client_.call(move_toward_srv_);
    }

    if ((fabs(distance_error) < dist_thresh) &&
        (fabs(theta_error) < theta_thresh)) {
      ROS_INFO("%s: Arrived", action_name_.c_str());
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
    as_.setSucceeded(result_);
  }

  move_init_client_.call(move_init_srv_);

}
