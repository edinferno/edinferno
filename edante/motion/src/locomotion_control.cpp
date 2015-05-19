/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI locomotion control methods
*/

#include "locomotion_control.h"

LocomotionControl::LocomotionControl(ros::NodeHandle* nh,
                                     AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Locomotion Control publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("is_moving", 10);
  INFO("Setting up Locomotion Control services" << std::endl);
  srv_move_ =
    nh_->advertiseService("move",
                          &LocomotionControl::move, this);
  srv_move_to_ =
    nh_->advertiseService("move_to",
                          &LocomotionControl::moveTo, this);
  srv_move_toward_ =
    nh_->advertiseService("move_toward",
                          &LocomotionControl::moveToward, this);
  srv_move_init_ =
    nh_->advertiseService("move_init",
                          &LocomotionControl::moveInit, this);
  srv_wait_move_finished_ =
    nh_->advertiseService("wait_move_finish",
                          &LocomotionControl::waitUntilMoveIsFinished, this);
  srv_stop_move_ =
    nh_->advertiseService("stop_move",
                          &LocomotionControl::stopMove, this);
  srv_get_move_config_ =
    nh_->advertiseService("get_move_config",
                          &LocomotionControl::getMoveConfig, this);
  srv_get_robot_position_ =
    nh_->advertiseService("get_robot_position",
                          &LocomotionControl::getRobotPosition, this);
  srv_get_next_robot_position_ =
    nh_->advertiseService("get_next_robot_position",
                          &LocomotionControl::getNextRobotPosition, this);
  srv_get_robot_velocity_ =
    nh_->advertiseService("get_robot_velocity",
                          &LocomotionControl::getRobotVelocity, this);
  srv_get_walk_arms_enabled_ =
    nh_->advertiseService("get_walk_arms_enabled",
                          &LocomotionControl::getWalkArmsEnabled, this);
  srv_set_walk_arms_enabled_ =
    nh_->advertiseService("set_walk_arms_enabled",
                          &LocomotionControl::setWalkArmsEnabled, this);
}

LocomotionControl::~LocomotionControl() {
  ros::shutdown();
}

bool LocomotionControl::move(motion::Move::Request &req,
                             motion::Move::Response &res) {
  // Check for size of move_configuration
  int configSize = req.move_configuration.names.size();
  AL::ALValue move_configuration;
  if (configSize > 0) {
    move_configuration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      move_configuration[i] = AL::ALValue::array(
                                req.move_configuration.names[i],
                                req.move_configuration.values[i]);
    }
  }

  res.res = true;
  if (configSize == 0) {
    mProxy_->move(req.target_velocity.x,
                  req.target_velocity.y,
                  req.target_velocity.theta);
  } else if (configSize > 0) {
    mProxy_->move(req.target_velocity.x,
                  req.target_velocity.y,
                  req.target_velocity.theta,
                  move_configuration);
  } else {
    res.res = false;
  }
  return true;
}

bool LocomotionControl::moveTo(motion::MoveTo::Request &req,
                               motion::MoveTo::Response &res) {
  //  Check for multiple positions
  int posNum = req.control_points.size();
  AL::ALValue control_points;
  if (posNum > 1) {
    control_points.arraySetSize(posNum);
    for (int i = 0; i < posNum; ++i) {
      control_points[i] = AL::ALValue::array(req.control_points[i].x,
                                             req.control_points[i].y,
                                             req.control_points[i].theta);
    }
  }

  // Check for size of move_configuration
  int configSize = req.move_configuration.names.size();
  AL::ALValue move_configuration;
  if (configSize > 0) {
    move_configuration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      move_configuration[i] = AL::ALValue::array(
                                req.move_configuration.names[i],
                                req.move_configuration.values[i]);
    }
  }

  res.res = true;
  if (posNum == 1 && configSize == 0) {
    mProxy_->post.moveTo(req.control_points[0].x,
                         req.control_points[0].y,
                         req.control_points[0].theta);
  } else if (posNum > 1 && configSize == 0) {
    mProxy_->post.moveTo(control_points);
  } else if (posNum == 1 && configSize > 0) {
    mProxy_->post.moveTo(req.control_points[0].x,
                         req.control_points[0].y,
                         req.control_points[0].theta,
                         move_configuration);
  } else if (posNum > 1 && configSize > 0) {
    mProxy_->post.moveTo(control_points, move_configuration);
  } else {
    res.res = false;
  }
  return true;
}

bool LocomotionControl::moveToward(motion::MoveToward::Request &req,
                                   motion::MoveToward::Response &res) {
  // Check for size of move_configuration
  int configSize = req.move_configuration.names.size();
  AL::ALValue move_configuration;
  if (configSize > 0) {
    move_configuration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      move_configuration[i] = AL::ALValue::array(
                                req.move_configuration.names[i],
                                req.move_configuration.values[i]);
    }
  }

  res.res = true;
  if (configSize == 0) {
    mProxy_->moveToward(req.norm_velocity.x,
                        req.norm_velocity.y,
                        req.norm_velocity.theta);
  } else if (configSize > 0) {
    mProxy_->moveToward(req.norm_velocity.x,
                        req.norm_velocity.y,
                        req.norm_velocity.theta,
                        move_configuration);
  } else {
    res.res = false;
  }
  return true;
}

bool LocomotionControl::moveInit(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res) {
  mProxy_->moveInit();
  return true;
}

bool LocomotionControl::waitUntilMoveIsFinished(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res) {
  mProxy_->waitUntilMoveIsFinished();
  return true;
}

bool LocomotionControl::moveIsActive() {
  return mProxy_->moveIsActive();
}

bool LocomotionControl::stopMove(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res) {
  mProxy_->stopMove();
  return true;
}

bool LocomotionControl::getMoveConfig(motion::GetMoveConfig::Request &req,
                                      motion::GetMoveConfig::Response &res) {
  AL::ALValue move_configuration;
  move_configuration = mProxy_->getMoveConfig(req.config);
  std::size_t CSize = move_configuration.getSize();
  res.move_configuration.names.resize(CSize);
  res.move_configuration.values.resize(CSize);
  for (size_t i = 0; i < CSize; ++i) {
    res.move_configuration.names[i] = move_configuration[i][0].toString();
    res.move_configuration.values[i] = move_configuration[i][1];
  }
  return true;
}

bool LocomotionControl::getRobotPosition(
  motion::GetRobotPosition::Request &req,
  motion::GetRobotPosition::Response &res) {
  res.positions = mProxy_->getRobotPosition(req.use_sensors);
  return true;
}

bool LocomotionControl::getNextRobotPosition(
  motion::GetNextRobotPosition::Request &req,
  motion::GetNextRobotPosition::Response &res) {
  res.positions = mProxy_->getNextRobotPosition();
  return true;
}

bool LocomotionControl::getRobotVelocity(
  motion::GetRobotVelocity::Request &req,
  motion::GetRobotVelocity::Response &res) {
  res.velocity = mProxy_->getRobotVelocity();
  return true;
}

bool LocomotionControl::getWalkArmsEnabled(
  motion::GetWalkArmsEnabled::Request &req,
  motion::GetWalkArmsEnabled::Response &res) {
  AL::ALValue result = mProxy_->getWalkArmsEnabled();
  res.arm_motions.resize(2);
  res.arm_motions[0] = static_cast<bool>(result[0]);
  res.arm_motions[1] = static_cast<bool>(result[1]);
  return true;
}

bool LocomotionControl::setWalkArmsEnabled(
  motion::SetWalkArmsEnabled::Request &req,
  motion::SetWalkArmsEnabled::Response &res) {
  mProxy_->setWalkArmsEnabled(req.left_arm_enable, req.right_arm_enable);
  return true;
}

void LocomotionControl::spinTopics() {
  std_msgs::Bool msg;
  msg.data = moveIsActive();
  moving_pub_.publish(msg);
}
