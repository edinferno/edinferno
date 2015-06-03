#include "navigation_server.h"

NavigateAction::NavigateAction(std::string name) :
  as_(nh_, name, boost::bind(&NavigateAction::executeCB, this, _1), false),
  action_name_(name) {
  get_pose_client_ = nh_.serviceClient<motion::GetRobotPosition>(
                       "/motion/get_robot_position", true);
  get_pose_client_.waitForExistence();
  move_to_client_ = nh_.serviceClient<motion::MoveTo>(
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
  ROS_INFO("Starting Navigation server");
  as_.start();
}

NavigateAction::~NavigateAction(void) {
}

void NavigateAction::executeCB(const navigation::NavigateGoalConstPtr &goal) {
  bool going = true;
  bool success = true;
  float thresh = 0.01f;
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  get_pose_client_.call(start_position_);

  // ROTATE FIRST, THEN STRAIGHT TO GOAL
  move_to_srv_.request.control_points.resize(2);
  move_to_srv_.request.control_points[0].x = 0.0f;
  move_to_srv_.request.control_points[0].y = 0.0f;
  move_to_srv_.request.control_points[0].theta = goal->target_pose.theta;
  move_to_srv_.request.control_points[1].x = goal->target_pose.x;
  move_to_srv_.request.control_points[1].y = 0.0f;
  move_to_srv_.request.control_points[1].theta = 0.0f;
  move_to_client_.call(move_to_srv_);
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
    ROS_INFO("Target pos: %f, %f, %f",
             goal->target_pose.x,
             goal->target_pose.y,
             goal->target_pose.theta);
    float theta_error = goal->target_pose.theta - feedback_.curr_pose.theta;
    float distance = sqrt(pow(feedback_.curr_pose.x, 2) +
                          pow(feedback_.curr_pose.y, 2) );
    float distance_error = goal->target_pose.x - distance;
    ROS_INFO("Theta error: %f, Distance_error: %f", theta_error, distance_error);
    if ((fabs(distance_error) < thresh) && (fabs(theta_error) < thresh)) {
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
