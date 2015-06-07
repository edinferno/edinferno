/**
 * @file      locomotion_control.cpp
 * @brief     ROS wrapper for NaoQI locomotion control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-18
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/locomotion_control.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

/**
 * @brief Locomotion class constructor
 * @details Called when Locomotion class is instantiated. Inheritance: **ALModule**
 *
 * @param broker ALBroker from class instantiation
 * @param name Name of class instantiation
 */
LocomotionControl::LocomotionControl(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Locomotion control module.");
}

/**
 * @brief Empty Locomotion destructor
 */
LocomotionControl::~LocomotionControl() {
}

void LocomotionControl::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

/**
 * @brief ROS setup function
 * @details Uses the passed ROS NodeHandle to advertise all LocomotionControl services/publisher.
 *
 * @param nh ROS NodeHandle, passed by Motion
 */
void LocomotionControl::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Locomotion Control publishers");
  moving_pub_ = nh_->advertise<std_msgs::Bool>("is_moving", 10, true);
  ROS_INFO_STREAM("Setting up Locomotion Control services");
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
  srv_move_is_active_ =
    nh_->advertiseService("move_is_active",
                          &LocomotionControl::moveIsActive, this);
  this->checkMoveActive();
}

/**
 * @brief Makes the robot move at the given velocity.
 * @details Expressed in `FRAME_ROBOT`, it takes an optional MoveConfiguration
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::move__floatCR.floatCR.floatCR)
 *
 * @param req.velocity            **motion_msgs::Velocity**
 * @param req.move_configuration  **motion_msgs::MoveConfiguration**
 * @param res.res bool            **std_msgs::Bool**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::move(motion_msgs::Move::Request& req,
                             motion_msgs::Move::Response& res) {
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
    mProxy_.move(req.target_velocity.x,
                 req.target_velocity.y,
                 req.target_velocity.theta);
  } else if (configSize > 0) {
    mProxy_.move(req.target_velocity.x,
                 req.target_velocity.y,
                 req.target_velocity.theta,
                 move_configuration);
  } else {
    res.res = false;
  }
  this->checkMoveActive();
  return true;
}

/**
 * @brief Makes NAO move to the given pose in the ground plane.
 * @details Relative to `FRAME_ROBOT`, it takes optional single/multiple control
 * points and optional single/multiple move configurations.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR)
 *
 * @param req.control_points      **motion_msgs::Position**
 * @param req.move_configuration  **motion_msgs::MoveConfiguration**
 * @param res.res bool            **std_msgs::Bool**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::moveTo(motion_msgs::MoveTo::Request& req,
                               motion_msgs::MoveTo::Response& res) {
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
    mProxy_.post.moveTo(req.control_points[0].x,
                        req.control_points[0].y,
                        req.control_points[0].theta);
  } else if (posNum > 1 && configSize == 0) {
    mProxy_.post.moveTo(control_points);
  } else if (posNum == 1 && configSize > 0) {
    mProxy_.post.moveTo(req.control_points[0].x,
                        req.control_points[0].y,
                        req.control_points[0].theta,
                        move_configuration);
  } else if (posNum > 1 && configSize > 0) {
    mProxy_.post.moveTo(control_points, move_configuration);
  } else {
    res.res = false;
  }
  this->checkMoveActive();
  return true;
}

/**
 * @brief Makes the robot move at the given normalized velocity.
 * @details Relative to `FRAME_ROBOT`, it takes an optional move configuration.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::moveToward__floatCR.floatCR.floatCR)
 *
 * @param req.norm_velocity       **motion_msgs::Velocity**
 * @param req.move_configuration  **motion_msgs::MoveConfiguration**
 * @param res.res                 **std_msgs::Bool**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::moveToward(motion_msgs::MoveToward::Request& req,
                                   motion_msgs::MoveToward::Response& res) {
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
    mProxy_.moveToward(req.norm_velocity.x,
                       req.norm_velocity.y,
                       req.norm_velocity.theta);
  } else if (configSize > 0) {
    mProxy_.moveToward(req.norm_velocity.x,
                       req.norm_velocity.y,
                       req.norm_velocity.theta,
                       move_configuration);
  } else {
    res.res = false;
  }
  this->checkMoveActive();
  return true;
}

/**
 * @brief Initializes the move process.
 * @details Checks the robot pose and takes a right posture. This is blocking called.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::moveInit)
 *
 * @param req **std_srvs::Empty**
 * @param res **std_srvs::Empty**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::moveInit(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Response& res) {
  mProxy_.post.moveInit();
  this->checkMoveActive();
  return true;
}

/**
 * @brief Waits until the MoveTask is finished.
 * @details This method can be used to block your script/code execution until the move task is totally finished.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::waitUntilMoveIsFinished)
 *
 * @param req **std_srvs::Empty**
 * @param res **std_srvs::Empty**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::waitUntilMoveIsFinished(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res) {
  mProxy_.waitUntilMoveIsFinished();
  this->checkMoveActive();
  return true;
}

/**
 * @brief Stops Move task at next double support.
 * @details This method will end the move task less brutally than killMove but quicker than `move(0.0, 0.0, 0.0)`.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::stopMove)
 *
 * @param req **std_srvs::Empty**
 * @param res **std_srvs::Empty**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::stopMove(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Response& res) {
  mProxy_.stopMove();
  this->checkMoveActive();
  return true;
}

/**
 * @brief Returns the move config.
 * @details Move configuration: `(“MaxStepX”, “MaxStepY”, “MaxStepTheta”, “MaxStepFrequency”, “StepHeight”, “TorsoWx”, “TorsoWy”)`
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::getMoveConfig__ssCR)
 *
 * @param req.config              **std_msgs::string**
 * @param res.move_configuration  **motion_msgs::MoveConfiguration**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::getMoveConfig(motion_msgs::GetMoveConfig::Request& req,
                                      motion_msgs::GetMoveConfig::Response& res) {
  AL::ALValue move_configuration;
  move_configuration = mProxy_.getMoveConfig(req.config);
  std::size_t CSize = move_configuration.getSize();
  res.move_configuration.names.resize(CSize);
  res.move_configuration.values.resize(CSize);
  for (size_t i = 0; i < CSize; ++i) {
    res.move_configuration.names[i] = move_configuration[i][0].toString();
    res.move_configuration.values[i] = move_configuration[i][1];
  }
  this->checkMoveActive();
  return true;
}

/**
 * @brief Gets the World Absolute Robot Position.
 * @details Returns a vector containing the World Absolute Robot Position. `(Absolute Position X, Absolute Position Y, Absolute Angle Theta (Wz))`
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::getRobotPosition__bCR)
 *
 * @param req.use_sensors **std_msgs::Bool** – If _true_, use the MRE sensor values
 * @param res.position  **motion_msgs::position**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::getRobotPosition(
  motion_msgs::GetRobotPosition::Request& req,
  motion_msgs::GetRobotPosition::Response& res) {
  std::vector<float> pose = mProxy_.getRobotPosition(req.use_sensors);
  res.position.x = pose[0];
  res.position.y = pose[1];
  res.position.theta = pose[2];
  this->checkMoveActive();
  return true;
}

/**
 * @brief Gets the World Absolute next Robot Position.
 * @details When no walk process active, getNextRobotPosition() = getRobotPosition().
 * Else getNextRobotPosition() returns the position of the robot after the unchangeable foot steps.
 * \n Returns a vector containing the World Absolute next Robot position. `(Absolute Position X, Absolute Position Y, Absolute Angle Theta (Wz))`
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::getNextRobotPosition)
 *
 * @param req                 **None**
 * @param res.next_position   **motion_msgs::Position**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::getNextRobotPosition(
  motion_msgs::GetNextRobotPosition::Request& req,
  motion_msgs::GetNextRobotPosition::Response& res) {
  std::vector<float> pose = mProxy_.getNextRobotPosition();
  res.next_position.x = pose[0];
  res.next_position.y = pose[1];
  res.next_position.theta = pose[2];
  this->checkMoveActive();
  return true;
}

/**
 * @brief Gets the World Absolute Robot Velocity.
 * @details Returns a vector containing the World Absolute Robot Velocity. `(Absolute Velocity Translation X [m.s-1], Absolute Velocity Translation Y[m.s-1], Absolute Velocity Rotation WZ [rd.s-1])`
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::getRobotVelocity)
 *
 * @param req [description]
 * @param res [description]
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::getRobotVelocity(
  motion_msgs::GetRobotVelocity::Request& req,
  motion_msgs::GetRobotVelocity::Response& res) {
  std::vector<float> velocity = mProxy_.getRobotVelocity();
  res.velocity.x = velocity[0];
  res.velocity.y = velocity[1];
  res.velocity.theta = velocity[2];
  this->checkMoveActive();
  return true;
}

/**
 * @brief Returns if Arms Motions are enabled during the Walk Process.
 * @details If _true_ Arms motions are controlled by the Walk Task.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::getWalkArmsEnabled)
 *
 * @param req             **None**
 * @param res.arm_motions **std_msgs::Bool[]**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::getWalkArmsEnabled(
  motion_msgs::GetWalkArmsEnabled::Request& req,
  motion_msgs::GetWalkArmsEnabled::Response& res) {
  AL::ALValue result = mProxy_.getWalkArmsEnabled();
  res.arm_motions.resize(2);
  res.arm_motions[0] = static_cast<bool>(result[0]);
  res.arm_motions[1] = static_cast<bool>(result[1]);
  this->checkMoveActive();
  return true;
}

/**
 * @brief Sets if Arms Motions are enabled during the Walk Process.
 * @details If _true_ Arms motions are controlled by the Walk Task.
 * \n [NAOqi function](http://doc.aldebaran.com/1-14/naoqi/motion/control-walk-api.html#ALMotionProxy::setWalkArmsEnabled__bCR.bCR)
 *
 * @param req.left_arm_enable   **std_msgs::Bool**
 * @param req.right_arm_enable  **std_msgs::Bool**
 * @param res                   **None**
 *
 * @return _true_ if service completed successfully
 */
bool LocomotionControl::setWalkArmsEnabled(
  motion_msgs::SetWalkArmsEnabled::Request& req,
  motion_msgs::SetWalkArmsEnabled::Response& res) {
  mProxy_.setWalkArmsEnabled(req.left_arm_enable, req.right_arm_enable);
  this->checkMoveActive();
  return true;
}

/**
 * @brief Publish whether current move is Active
 * @details This function is called automatically after any LocomotionControl srv call */
void LocomotionControl::checkMoveActive() {
  move_active.data = mProxy_.moveIsActive();
  moving_pub_.publish(move_active);
}

/**
 * @brief Responde whether current move is Active
 * @details This is a service call instead of publisher */
bool LocomotionControl::moveIsActive(
  motion_msgs::IsEnabled::Request& req,
  motion_msgs::IsEnabled::Response& res) {
  res.is_enabled = mProxy_.moveIsActive();
  return true;
}
