#include "navigation/search_for_ball.hpp"

SearchForBallAction::SearchForBallAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&SearchForBallAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&SearchForBallAction::preemptCB, this));
  ball_pos_sub_ = nh_.subscribe("/vision/ball", 1,
                                &SearchForBallAction::ballCB, this);
  scan_client_ = nh_.serviceClient<motion_msgs::AngleInterp>(
                   "/motion/angle_interp", true);
  scan_client_.waitForExistence();
  turn_client_ = nh_.serviceClient<motion_msgs::MoveTo>(
                   "/motion/move_to", true);
  turn_client_.waitForExistence();
  camera_client_ = nh_.serviceClient<camera_msgs::SetActiveCamera>(
                     "/camera/set_active_camera", true);
  camera_client_.waitForExistence();
  resource_avail_client_ = nh_.serviceClient<motion_msgs::AreResourcesAvailable>(
                             "/motion/are_resources_available", true);
  resource_avail_client_.waitForExistence();
  move_is_active_client_ = nh_.serviceClient<motion_msgs::IsEnabled>(
                             "/motion/move_is_active", true);
  move_is_active_client_.waitForExistence();
  kill_task_client_ = nh_.serviceClient<motion_msgs::TaskResource>(
                        "/motion/kill_tasks_using_resources", true);
  kill_task_client_.waitForExistence();
  this->init();
  ROS_INFO("Starting SearchForBall server");
  as_.start();
}

SearchForBallAction::~SearchForBallAction(void) {
}

void SearchForBallAction::init() {
  // Variables setup
  WAIT_100MS_ = 100000;
  N_SCANS_ = 1000;
  scan_no_ = 0;

  // Flags init
  ball_found_ = false;
  scanning_right_ = false;
  scanning_left_ = false;
  turning_ = false;
  time_out_ = false;

  resource_avail_srv_.request.resource_names.push_back("HeadYaw");
  kill_task_srv_.request.resource_names.push_back("HeadYaw");

  // Prepare look left srv
  look_left_srv_.request.names.push_back("HeadYaw");
  look_left_srv_.request.names.push_back("HeadPitch");
  look_left_srv_.request.angle_lists.resize(2);
  look_left_srv_.request.angle_lists[0].float_list.push_back(HALF_PI);
  look_left_srv_.request.angle_lists[1].float_list.push_back(0.0f);
  look_left_srv_.request.time_lists.resize(2);
  look_left_srv_.request.time_lists[0].float_list.push_back(0.5f);
  look_left_srv_.request.time_lists[1].float_list.push_back(0.5f);
  look_left_srv_.request.is_absolute = true;
  // Prepare scan right srv
  scan_right_srv_.request.names.push_back("HeadYaw");
  scan_right_srv_.request.names.push_back("HeadPitch");
  scan_right_srv_.request.angle_lists.resize(2);
  scan_right_srv_.request.angle_lists[0].float_list.push_back(-HALF_PI);
  scan_right_srv_.request.angle_lists[1].float_list.push_back(0.0f);
  scan_right_srv_.request.time_lists.resize(2);
  scan_right_srv_.request.time_lists[0].float_list.push_back(4.0f);
  scan_right_srv_.request.time_lists[1].float_list.push_back(4.0f);
  scan_right_srv_.request.is_absolute = true;
  // Prepare scan left srv
  scan_left_srv_.request.names.push_back("HeadYaw");
  scan_left_srv_.request.names.push_back("HeadPitch");
  scan_left_srv_.request.angle_lists.resize(2);
  scan_left_srv_.request.angle_lists[0].float_list.push_back(HALF_PI);
  scan_left_srv_.request.angle_lists[1].float_list.push_back(0.0f);
  scan_left_srv_.request.time_lists.resize(2);
  scan_left_srv_.request.time_lists[0].float_list.push_back(4.0f);
  scan_left_srv_.request.time_lists[1].float_list.push_back(4.0f);
  scan_left_srv_.request.is_absolute = true;
  // Prepare look straight srv
  look_straight_srv_.request.names.push_back("HeadYaw");
  look_straight_srv_.request.names.push_back("HeadPitch");
  look_straight_srv_.request.angle_lists.resize(2);
  look_straight_srv_.request.angle_lists[0].float_list.push_back(0.0f);
  look_straight_srv_.request.angle_lists[1].float_list.push_back(0.0f);
  look_straight_srv_.request.time_lists.resize(2);
  look_straight_srv_.request.time_lists[0].float_list.push_back(0.5f);
  look_straight_srv_.request.time_lists[1].float_list.push_back(0.5f);
  look_straight_srv_.request.is_absolute = true;
  // Prepare turn left srv
  turn_left_srv_.request.control_points.resize(1);
  turn_left_srv_.request.control_points[0].x = 0.0;
  turn_left_srv_.request.control_points[0].y = 0.0;
  turn_left_srv_.request.control_points[0].theta = HALF_PI;
  // Prepare turn right srv
  turn_right_srv_.request.control_points.resize(1);
  turn_right_srv_.request.control_points[0].x = 0.0;
  turn_right_srv_.request.control_points[0].y = 0.0;
  turn_right_srv_.request.control_points[0].theta = -HALF_PI;

  // Prepare camera srvs
  bottom_camera_srv_.request.active_camera = 1;
  top_camera_srv_.request.active_camera = 0;
}

void SearchForBallAction::ballCB(const vision_msgs::BallDetection::ConstPtr&
                                 msg) {
  ball_found_ = msg->is_detected;
  // ball_pos_.x = msg->pos_robot.x;
  // ball_pos_.y = msg->pos_robot.y;
}

void SearchForBallAction::goalCB() {
  as_.acceptNewGoal();
  going_ = true;
  this->executeCB();
}

void SearchForBallAction::preemptCB() {
  going_ = false;
  as_.setPreempted();
}

void SearchForBallAction::scan_right() {
  camera_client_.call(bottom_camera_srv_);
  scan_client_.call(look_left_srv_);
  scan_client_.call(scan_right_srv_);
}

void SearchForBallAction::scan_left() {
  camera_client_.call(top_camera_srv_);
  scan_client_.call(scan_left_srv_);
}

void SearchForBallAction::executeCB() {
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  move_init_client_.call(move_init_srv_);

  while (going_) {
    resource_avail_client_.call(resource_avail_srv_);
    move_is_active_client_.call(move_is_active_srv_);

    // START SEARCH
    if (scan_no_ < N_SCANS_) {
      if (!scanning_right_ && !scanning_left_ && !turning_) {
        // START SCAN RIGHT
        scanning_right_ = true;
        this->scan_right();
      } else if (scanning_right_ && resource_avail_srv_.response.available) {
        // START SCAN LEFT
        scanning_right_ = false;
        scanning_left_ = true;
        this->scan_left();
      } else if (scanning_left_ && resource_avail_srv_.response.available) {
        // START TURNING
        scanning_left_ = false;
        turning_ = true;
        turn_client_.call(turn_right_srv_);
        scan_client_.call(look_straight_srv_);
      } else if (turning_ && !move_is_active_srv_.response.is_enabled) {
        // SEARCH FINISHED
        scan_no_++;
        turning_ = false;
      }
    } else {
      // Number of maximum scans reached
      going_ = false;
      time_out_ = true;
    }

    // Check if we have found the ball
    if (ball_found_) {
      ROS_INFO("%s: Ball Found!", action_name_.c_str());
      success = true;
      going_ = false;
    }
    ros::spinOnce();
    r.sleep();
  }

  kill_task_client_.call(kill_task_srv_);
  // stop_move_client_.call(stop_move_srv_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    if (time_out_) {ROS_INFO("Timed out!");}
    else if (as_.isPreemptRequested()) {ROS_INFO("Search preempted");}
    else if (!ball_found_) {ROS_INFO("Ball not found!");}
    as_.setAborted(result_);
  }
  // move_init_client_.call(move_init_srv_);
}
