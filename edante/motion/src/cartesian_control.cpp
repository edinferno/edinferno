/**
 * @file      cartesian_control.cpp
 * @brief     ROS wrapper for NaoQI cartesian control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/cartesian_control.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

CartesianControl::CartesianControl(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Cartesian Control module.");
}

CartesianControl::~CartesianControl() {
}

void CartesianControl::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void CartesianControl::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Cartesian Control services");
  srv_position_interpolation_ =
    nh_->advertiseService("position_interpolation",
                          &CartesianControl::positionInterpolation, this);
  srv_position_interpolations_ =
    nh_->advertiseService("position_interpolations",
                          &CartesianControl::positionInterpolations, this);
  srv_set_position_ =
    nh_->advertiseService("set_position",
                          &CartesianControl::setPosition, this);
  srv_change_position_ =
    nh_->advertiseService("change_position",
                          &CartesianControl::changePosition, this);
  srv_get_position_ =
    nh_->advertiseService("get_position",
                          &CartesianControl::getPosition, this);
  srv_get_transform_ =
    nh_->advertiseService("get_transform",
                          &CartesianControl::getTransform, this);
}

bool CartesianControl::positionInterpolation(
  motion_msgs::PositionInterpolation::Request& req,
  motion_msgs::PositionInterpolation::Response& res) {
  AL::ALValue traj_points;
  size_t l = req.path.traj_points.size();
  traj_points.arraySetSize(l);
  for (size_t i = 0; i < l; ++i) {
    size_t v = req.path.traj_points[i].float_list.size();
    for (size_t ii = 0; ii < v; ++ii) {
      traj_points[i].arrayPush(req.path.traj_points[i].float_list[ii]);
    }
  }
  mProxy_.positionInterpolation(req.chain_name, req.space, traj_points,
                                req.axis_mask, req.durations, req.is_absolute);
  return true;
}

bool CartesianControl::positionInterpolations(
  motion_msgs::PositionInterpolations::Request& req,
  motion_msgs::PositionInterpolations::Response& res) {
  size_t s = req.paths.size();
  AL::ALValue paths;
  paths.arraySetSize(s);
  for (size_t i = 0; i < s; ++i) {
    size_t l = req.paths[i].traj_points.size();
    paths[i].arraySetSize(l);
    for (size_t ii = 0; ii < l; ++ii) {
      size_t v = req.paths[i].traj_points[ii].float_list.size();
      for (size_t iii = 0; iii < v; ++iii) {
        paths[i][ii].arrayPush(
          req.paths[i].traj_points[ii].float_list[iii]);
      }
    }
  }

  s = req.durations.size();
  AL::ALValue durations;
  durations.arraySetSize(s);
  for (size_t i = 0; i < s; ++i) {
    size_t l = req.durations[i].float_list.size();
    for (size_t ii = 0; ii < l; ++ii) {
      durations[i].arrayPush(req.durations[i].float_list[ii]);
    }
  }
  mProxy_.positionInterpolations(req.effector_names, req.space, paths,
                                 req.axis_masks, durations, req.is_absolute);
  return true;
}

bool CartesianControl::setPosition(motion_msgs::SetPosition::Request& req,
                                   motion_msgs::SetPosition::Response& res) {
  mProxy_.setPosition(req.chain_name, req.space, req.position,
                      req.fraction_max_speed, req.axis_mask);
  return true;
}

bool CartesianControl::changePosition(
  motion_msgs::ChangePosition::Request& req,
  motion_msgs::ChangePosition::Response& res) {
  mProxy_.changePosition(req.effector_name, req.space, req.position_change,
                         req.fraction_max_speed, req.axis_mask);
  return true;
}

bool CartesianControl::getPosition(motion_msgs::GetPosition::Request& req,
                                   motion_msgs::GetPosition::Response& res) {
  res.position = mProxy_.getPosition(
                   req.name, req.space, req.use_sensor_values);
  return true;
}

bool CartesianControl::getTransform(motion_msgs::GetTransform::Request& req,
                                    motion_msgs::GetTransform::Response& res) {
  res.transform = mProxy_.getTransform(req.name, req.space,
                                       req.use_sensor_values);
  return true;
}
