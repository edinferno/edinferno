#include "locomotion_control.h"

Locomotion_Control::Locomotion_Control(ros::NodeHandle* nh,
                                       AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Nao locomotion publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("isMoving", 10);
  INFO("Setting up Nao motion publishers" << std::endl);
  srv_move_ = nh_->advertiseService("move", &Locomotion_Control::move, this);
  srv_move_to_ = nh_->advertiseService("moveTo",
                                       &Locomotion_Control::moveTo, this);
  srv_move_toward_ = nh_->advertiseService("moveToward",
                     &Locomotion_Control::moveToward, this);
  srv_move_init_ = nh_->advertiseService("moveInit",
                                         &Locomotion_Control::moveInit, this);
  srv_wait_move_finished_ = nh_->advertiseService("waitMoveFinish",
                            &Locomotion_Control::waitUntilMoveIsFinished, this);
  srv_stop_move_ = nh_->advertiseService("stopMove",
                                         &Locomotion_Control::stopMove, this);
  srv_get_move_config_ = nh_->advertiseService("getMoveConfig",
                         &Locomotion_Control::getMoveConfig, this);
  srv_get_robot_position_ = nh_->advertiseService("getRobotPosition",
                            &Locomotion_Control::getRobotPosition, this);
  srv_get_next_robot_position_ = nh_->advertiseService("getNextRobotPosition",
                                 &Locomotion_Control::getNextRobotPosition, this);
  srv_get_robot_velocity_ = nh_->advertiseService("getRobotVelocity",
                            &Locomotion_Control::getRobotVelocity, this);
  srv_get_walk_arms_enabled_ = nh_->advertiseService("getWalkArmsEnabled",
                               &Locomotion_Control::getWalkArmsEnabled, this);
  srv_set_walk_arms_enabled_ = nh_->advertiseService("setWalkArmsEnabled",
                               &Locomotion_Control::setWalkArmsEnabled, this);
}

Locomotion_Control::~Locomotion_Control() {
  ros::shutdown();
}

bool Locomotion_Control::move(motion::move::Request &req,
                              motion::move::Response &res) {
  // Check for size of moveConfiguration
  int configSize = req.moveConfiguration.names.size();
  AL::ALValue moveConfiguration;
  if (configSize > 0) {
    moveConfiguration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      moveConfiguration[i] = AL::ALValue::array(
                               req.moveConfiguration.names[i],
                               req.moveConfiguration.values[i]);
    }
  }

  res.res = true;
  if (configSize == 0) {
    mProxy_->move(req.targetVelocity.x,
                  req.targetVelocity.y,
                  req.targetVelocity.theta);
  } else if (configSize > 0) {
    mProxy_->move(req.targetVelocity.x,
                  req.targetVelocity.y,
                  req.targetVelocity.theta,
                  moveConfiguration);
  } else {
    res.res = false;
  }
  return true;
}

bool Locomotion_Control::moveTo(motion::moveTo::Request &req,
                                motion::moveTo::Response &res) {
  //  Check for multiple positions
  int posNum = req.controlPoints.size();
  AL::ALValue controlPoints;
  if (posNum > 1) {
    controlPoints.arraySetSize(posNum);
    for (int i = 0; i < posNum; ++i) {
      controlPoints[i] = AL::ALValue::array(req.controlPoints[i].x,
                                            req.controlPoints[i].y,
                                            req.controlPoints[i].theta);
    }
  }

  // Check for size of moveConfiguration
  int configSize = req.moveConfiguration.names.size();
  AL::ALValue moveConfiguration;
  if (configSize > 0) {
    moveConfiguration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      moveConfiguration[i] = AL::ALValue::array(
                               req.moveConfiguration.names[i],
                               req.moveConfiguration.values[i]);
    }
  }

  res.res = true;
  if (posNum == 1 && configSize == 0) {
    mProxy_->post.moveTo(req.controlPoints[0].x,
                         req.controlPoints[0].y,
                         req.controlPoints[0].theta);
  } else if (posNum > 1 && configSize == 0) {
    mProxy_->post.moveTo(controlPoints);
  } else if (posNum == 1 && configSize > 0) {
    mProxy_->post.moveTo(req.controlPoints[0].x,
                         req.controlPoints[0].y,
                         req.controlPoints[0].theta,
                         moveConfiguration);
  } else if (posNum > 1 && configSize > 0) {
    mProxy_->post.moveTo(controlPoints, moveConfiguration);
  } else {
    res.res = false;
  }
  return true;
}

bool Locomotion_Control::moveToward(motion::moveToward::Request &req,
                                    motion::moveToward::Response &res) {
  // Check for size of moveConfiguration
  int configSize = req.moveConfiguration.names.size();
  AL::ALValue moveConfiguration;
  if (configSize > 0) {
    moveConfiguration.arraySetSize(configSize);
    for (int i = 0; i < configSize; ++i) {
      moveConfiguration[i] = AL::ALValue::array(
                               req.moveConfiguration.names[i],
                               req.moveConfiguration.values[i]);
    }
  }

  res.res = true;
  if (configSize == 0) {
    mProxy_->moveToward(req.normVelocity.x,
                        req.normVelocity.y,
                        req.normVelocity.theta);
  } else if (configSize > 0) {
    mProxy_->moveToward(req.normVelocity.x,
                        req.normVelocity.y,
                        req.normVelocity.theta,
                        moveConfiguration);
  } else {
    res.res = false;
  }
  return true;
}

bool Locomotion_Control::moveInit(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res) {
  mProxy_->moveInit();
  return true;
}

bool Locomotion_Control::waitUntilMoveIsFinished(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res) {
  mProxy_->waitUntilMoveIsFinished();
  return true;
}

bool Locomotion_Control::moveIsActive() {
  mProxy_->moveIsActive();
  return true;
}

bool Locomotion_Control::stopMove(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res) {
  mProxy_->stopMove();
  return true;
}

bool Locomotion_Control::getMoveConfig(motion::getMoveConfig::Request &req,
                                       motion::getMoveConfig::Response &res) {
  AL::ALValue moveConfiguration;
  moveConfiguration = mProxy_->getMoveConfig(req.config);
  std::size_t CSize = moveConfiguration.getSize();
  res.moveConfiguration.names.resize(CSize);
  res.moveConfiguration.values.resize(CSize);
  for (int i = 0; i < CSize; ++i) {
    res.moveConfiguration.names[i] = moveConfiguration[i][0].toString();
    res.moveConfiguration.values[i] = moveConfiguration[i][1];
  }
  return true;
}

bool Locomotion_Control::getRobotPosition(
  motion::getRobotPosition::Request &req,
  motion::getRobotPosition::Response &res) {
  bool useSensors = req.useSensors;
  res.positions = mProxy_->getRobotPosition(useSensors);
  return true;
}

bool Locomotion_Control::getNextRobotPosition(
  motion::getNextRobotPosition::Request &req,
  motion::getNextRobotPosition::Response &res) {
  res.positions = mProxy_->getNextRobotPosition();
  return true;
}

bool Locomotion_Control::getRobotVelocity(
  motion::getRobotVelocity::Request &req,
  motion::getRobotVelocity::Response &res) {
  res.velocity = mProxy_->getRobotVelocity();
  return true;
}

bool Locomotion_Control::getWalkArmsEnabled(
  motion::getWalkArmsEnabled::Request &req,
  motion::getWalkArmsEnabled::Response &res) {
  AL::ALValue result = mProxy_->getWalkArmsEnabled();
  res.armMotions.resize(2);
  res.armMotions[0] = static_cast<bool>(result[0]);
  res.armMotions[1] = static_cast<bool>(result[1]);
  return true;
}

bool Locomotion_Control::setWalkArmsEnabled(
  motion::setWalkArmsEnabled::Request &req,
  motion::setWalkArmsEnabled::Response &res) {
  bool left = req.leftArmEnable;
  bool right = req.rightArmEnable;
  mProxy_->setWalkArmsEnabled(left, right);
  return true;
}

void Locomotion_Control::spinTopics() {
  std_msgs::Bool msg;
  msg.data = moveIsActive();
  moving_pub_.publish(msg);
}
