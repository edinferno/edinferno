/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI cartesian control methods
*/

#include "cartesian_control.h"

CartesianControl::CartesianControl(ros::NodeHandle* nh,
                                   AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Cartesian Control services" << std::endl);
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

CartesianControl::~CartesianControl() {
  ros::shutdown();
}

bool CartesianControl::positionInterpolation(
  motion::PositionInterpolation::Request &req,
  motion::PositionInterpolation::Response &res) {
  AL::ALValue traj_points;
  size_t l = req.path.traj_points.size();
  traj_points.arraySetSize(l);
  for (size_t i = 0; i < l; ++i) {
    size_t v = req.path.traj_points[i].float_list.size();
    for (size_t ii = 0; ii < v; ++ii) {
      traj_points[i].arrayPush(req.path.traj_points[i].float_list[ii]);
    }
  }
  mProxy_->positionInterpolation(req.chain_name, req.space, traj_points,
                                 req.axis_mask, req.durations, req.is_absolute);
  return true;
}

bool CartesianControl::positionInterpolations(
  motion::PositionInterpolations::Request &req,
  motion::PositionInterpolations::Response &res) {
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
  mProxy_->positionInterpolations(req.effector_names, req.space, paths,
                                  req.axis_masks, durations, req.is_absolute);
  return true;
}

bool CartesianControl::setPosition(motion::SetPosition::Request &req,
                                   motion::SetPosition::Response &res) {
  mProxy_->setPosition(req.chain_name, req.space, req.position,
                       req.fraction_max_speed, req.axis_mask);
  return true;
}

bool CartesianControl::changePosition(motion::ChangePosition::Request &req,
                                      motion::ChangePosition::Response &res) {
  mProxy_->changePosition(req.effector_name, req.space, req.position_change,
                          req.fraction_max_speed, req.axis_mask);
  return true;
}

bool CartesianControl::getPosition(motion::GetPosition::Request &req,
                                   motion::GetPosition::Response &res) {
  res.position = mProxy_->getPosition(req.name, req.space, req.use_sensor_values);
  return true;
}

bool CartesianControl::getTransform(motion::GetTransform::Request &req,
                                    motion::GetTransform::Response &res) {
  res.transform = mProxy_->getTransform(req.name, req.space,
                                        req.use_sensor_values);
  return true;
}
