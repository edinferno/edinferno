#include "navigation/move_to.hpp"

MoveToAction::MoveToAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, boost::bind(&MoveToAction::executeCB, this, _1), false),
  action_name_(name) {
  get_pose_client_ = nh_.serviceClient<motion_msgs::GetRobotPosition>(
                       "/motion/get_robot_position", true);
  get_pose_client_.waitForExistence();
  move_to_client_ = nh_.serviceClient<motion_msgs::MoveTo>(
                      "/motion/move_to", true);
  move_to_client_.waitForExistence();
  stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/stop_move", true);
  stop_move_client_.waitForExistence();
  move_init_client_ = nh_.serviceClient<std_srvs::Empty>(
                        "/motion/move_init", true);
  move_init_client_.waitForExistence();
  start_position_.request.use_sensors = true;
  get_pose_srv_.request.use_sensors = true;
  this->init();
  ROS_INFO("Starting Navigation server");
  as_.start();
}

MoveToAction::~MoveToAction(void) {
}

void MoveToAction::init() {
  thresh = 0.01f;
  move_to_srv_.request.control_points.resize(2);
  move_to_srv_.request.control_points[0].x = 0.0f;
  move_to_srv_.request.control_points[0].y = 0.0f;
  move_to_srv_.request.control_points[0].theta = 0.0f;
  move_to_srv_.request.control_points[1].x = 0.0f;
  move_to_srv_.request.control_points[1].y = 0.0f;
  move_to_srv_.request.control_points[1].theta = 0.0f;
}

void MoveToAction::executeCB(
  const navigation_msgs::MoveToGoalConstPtr& goal) {
  bool going = true;
  bool success = true;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  get_pose_client_.call(start_position_);

  // ROTATE FIRST, THEN STRAIGHT TO GOAL
  move_to_srv_.request.control_points[0].theta = goal->target_pose.theta;
  move_to_srv_.request.control_points[1].x = goal->target_pose.x;
  move_to_client_.call(move_to_srv_);

  while (going == true) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
      going = false;
    }
    get_pose_client_.call(get_pose_srv_);

    // Publish Feedback
    feedback_.curr_pose.x =
      get_pose_srv_.response.position.x - start_position_.response.position.x;
    feedback_.curr_pose.y =
      get_pose_srv_.response.position.y - start_position_.response.position.y;
    feedback_.curr_pose.theta =
      get_pose_srv_.response.position.theta - start_position_.response.position.theta;
    as_.publishFeedback(feedback_);

    // Calculate distance and orientation error
    float theta_error = goal->target_pose.theta - feedback_.curr_pose.theta;
    float distance = sqrt(pow(feedback_.curr_pose.x, 2) +
                          pow(feedback_.curr_pose.y, 2) );
    float distance_error = goal->target_pose.x - distance;

    // Check if robot has arrived
    if ((fabs(distance_error) < thresh) && (fabs(theta_error) < thresh)) {
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
