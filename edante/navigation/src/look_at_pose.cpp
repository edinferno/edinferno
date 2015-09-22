#include "navigation/look_at_pose.hpp"

LookAtPoseAction::LookAtPoseAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&LookAtPoseAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&LookAtPoseAction::preemptCB, this));
  ball_pos_sub_ = nh_.subscribe("/vision/ball", 1,
                                &LookAtPoseAction::ballCB, this);
  look_client_ = nh_.serviceClient<motion_msgs::SetAngles>(
                   "/motion/set_angles", true);
  look_client_.waitForExistence();
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
  monitor_client_ = nh_.serviceClient<motion_planning_msgs::MonitorMode>(
                      "/motion_planning/monitor_mode", true);
  monitor_client_.waitForExistence();
  curr_pose_client_ = nh_.serviceClient<localization_msgs::GetRobotPose>(
                        "/localization/get_robot_pose", true);
  curr_pose_client_.waitForExistence();
  this->init();
  ROS_INFO("Starting LookAtPose server");
  as_.start();
}

LookAtPoseAction::~LookAtPoseAction(void) {
}

void LookAtPoseAction::init() {
  // Torso normalized pose for vector angle calculation
  torso_norm_pose_.x = 1.0f;
  torso_norm_pose_.y = 0.0f;

  // Flags init
  ball_found_ = false;

  resource_avail_srv_.request.resource_names.push_back("HeadYaw");
  kill_task_srv_.request.resource_names.push_back("HeadYaw");

  // Prepare camera srvs
  // bottom_camera_srv_.request.active_camera = 1;
  top_camera_srv_.request.active_camera = 0;
  look_pose_srv_.request.names.push_back("HeadYaw");
  look_pose_srv_.request.names.push_back("HeadPitch");
  look_pose_srv_.request.angles.resize(2);
  look_pose_srv_.request.angles[1] = 0.0f;
  look_pose_srv_.request.fraction_max_speed = 1.0f;

  // Prepare monitor srvs
  // bottom_camera_monitor_srv_.request.monitor_mode = MonitorMode::BOTTOM_CAMERA;
  top_camera_monitor_srv_.request.monitor_mode = MonitorMode::TOP_CAMERA;
  ball_seen_monitor_srv_.request.monitor_mode = MonitorMode::BALL_SEEN;
  ball_lost_monitor_srv_.request.monitor_mode = MonitorMode::BALL_LOST;
}

void LookAtPoseAction::ballCB(const vision_msgs::BallDetection::ConstPtr&
                              msg) {
  ball_found_ = msg->is_detected;
  // ball_pos_.x = msg->pos_robot.x;
  // ball_pos_.y = msg->pos_robot.y;
}

void LookAtPoseAction::goalCB() {
  target_pose_ = as_.acceptNewGoal()->target_pose;
  this->executeCB();
}

void LookAtPoseAction::preemptCB() {
  going_ = false;
  as_.setPreempted();
}

void LookAtPoseAction::executeCB() {
  going_ = true;
  ros::Rate r(5);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  move_init_client_.call(move_init_srv_);

  kill_task_client_.call(kill_task_srv_);
  camera_client_.call(top_camera_srv_);
  monitor_client_.call(top_camera_monitor_srv_);

  while (going_) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going_ = false;
    }
    curr_pose_client_.call(curr_pose_srv_);
    curr_robot_pose_ = curr_pose_srv_.response.pose;
    rel_target_pose_.x = target_pose_.x - curr_robot_pose_.x;
    rel_target_pose_.y = target_pose_.y - curr_robot_pose_.y;
    // Calc angle from torso to targ pose
    float dot_prod = (torso_norm_pose_.x * rel_target_pose_.x) +
                     (torso_norm_pose_.y * rel_target_pose_.y);
    float mag_torso = sqrt(pow(torso_norm_pose_.x, 2) + pow(torso_norm_pose_.y, 2));
    float mag_targ = sqrt(pow(rel_target_pose_.x, 2) + pow(rel_target_pose_.y, 2));
    float rel_angle_ = acos(dot_prod / (mag_torso * mag_targ));
    float abs_angle_ = rel_angle_ - curr_robot_pose_.theta;
    if (rel_target_pose_.y < 0) { // If target is towards right of robot
      abs_angle_ = abs_angle_ * -1;
    }

    // float abs_angle_deg_ = (abs_angle_ * 180) / PI;
    // ROS_INFO("Head Angle Deg: %f", abs_angle_deg_);
    // Prepare look srv
    look_pose_srv_.request.angles[0] = abs_angle_;

    look_client_.call(look_pose_srv_);
    // resource_avail_client_.call(resource_avail_srv_);
    // move_is_active_client_.call(move_is_active_srv_);

    // Check if we have found the ball
    if (ball_found_) {
      monitor_client_.call(ball_seen_monitor_srv_);
    } else if (!ball_found_) {
      monitor_client_.call(ball_lost_monitor_srv_);
    }
    ros::spinOnce();
    r.sleep();
  }
  kill_task_client_.call(kill_task_srv_);
  // stop_move_client_.call(stop_move_srv_);

  result_.outcome = "preempted";
  if (as_.isPreemptRequested()) {ROS_INFO("Look preempted");}
  as_.setSucceeded(result_);

  // move_init_client_.call(move_init_srv_);
}
