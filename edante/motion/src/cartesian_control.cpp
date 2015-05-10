#include "cartesian_control.h"

Cartesian_Control::Cartesian_Control(ros::NodeHandle* nh,
                                     AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Cartesian Control services" << std::endl);
  srv_position_interpolation_ = nh_->advertiseService("positionInterpolation",
                                &Cartesian_Control::positionInterpolation, this);
  srv_position_interpolations_ = nh_->advertiseService("positionInterpolations",
                                 &Cartesian_Control::positionInterpolations, this);
  srv_set_position_ = nh_->advertiseService("setPosition",
                      &Cartesian_Control::setPosition, this);
  srv_change_position_ = nh_->advertiseService("changePosition",
                         &Cartesian_Control::changePosition, this);
  srv_get_position_ = nh_->advertiseService("getPosition",
                      &Cartesian_Control::getPosition, this);
  // srv_transform_interpolation_ = nh_->advertiseService("transformInterpolation",
  //                                &Cartesian_Control::transformInterpolation, this);
  // srv_set_transform_ = nh_->advertiseService("setTransform",
  //                      &Cartesian_Control::setTransform, this);
  // srv_change_transform_ = nh_->advertiseService("changeTransform",
  //                         &Cartesian_Control::changeTransform, this);
  srv_get_transform_ = nh_->advertiseService("getTransform",
                       &Cartesian_Control::getTransform, this);
}

Cartesian_Control::~Cartesian_Control() {
  ros::shutdown();
}

bool Cartesian_Control::positionInterpolation(
  motion::positionInterpolation::Request &req,
  motion::positionInterpolation::Response &res) {

  string chainName = req.chainName;
  int space = req.space;

  AL::ALValue trajPoints;
  size_t l = req.path.trajPoints.size();
  trajPoints.arraySetSize(l);
  for (size_t i = 0; i < l; ++i) {
    size_t v = req.path.trajPoints[i].floatList.size();
    for (size_t ii = 0; ii < v; ++ii) {
      trajPoints[i].arrayPush(req.path.trajPoints[i].floatList[ii]);
    }
  }

  int axisMask = req.axisMask;
  AL::ALValue durations = req.durations;
  bool isAbsolute = req.isAbsolute;
  mProxy_->positionInterpolation(chainName, space, trajPoints, axisMask,
                                 durations, isAbsolute);
  return true;
}

bool Cartesian_Control::positionInterpolations(
  motion::positionInterpolations::Request &req,
  motion::positionInterpolations::Response &res) {

  std::vector<string> effectorNames = req.effectorNames;
  int space = req.space;

  size_t s = req.paths.size();
  AL::ALValue paths;
  paths.arraySetSize(s);
  for (size_t i = 0; i < s; ++i) {
    size_t l = req.paths[i].trajPoints.size();
    paths[i].arraySetSize(l);
    for (size_t ii = 0; ii < l; ++ii) {
      size_t v = req.paths[i].trajPoints[ii].floatList.size();
      for (size_t iii = 0; iii < v; ++iii) {
        paths[i][ii].arrayPush(
          req.paths[i].trajPoints[ii].floatList[iii]);
      }
    }
  }

  AL::ALValue axisMasks = req.axisMasks;

  s = req.durations.size();
  AL::ALValue durations;
  durations.arraySetSize(s);
  for (size_t i = 0; i < s; ++i) {
    size_t l = req.durations[i].floatList.size();
    for (size_t ii = 0; ii < l; ++ii) {
      durations[i].arrayPush(req.durations[i].floatList[ii]);
    }
  }
  bool isAbsolute = req.isAbsolute;

  mProxy_->positionInterpolations(effectorNames, space, paths, axisMasks,
                                  durations, isAbsolute);
  return true;
}

bool Cartesian_Control::setPosition(motion::setPosition::Request &req,
                                    motion::setPosition::Response &res) {

  string chainName = req.chainName;
  int space = req.space;
  std::vector<float> position = req.position;
  float fractionMaxSpeed = req.fractionMaxSpeed;
  int axisMask = req.axisMask;
  mProxy_->setPosition(chainName, space, position, fractionMaxSpeed, axisMask);
  return true;
}

bool Cartesian_Control::changePosition(motion::changePosition::Request &req,
                                       motion::changePosition::Response &res) {

  string effectorName = req.effectorName;
  int space = req.space;
  std::vector<float> positionChange = req.positionChange;
  float fractionMaxSpeed = req.fractionMaxSpeed;
  int axisMask = req.axisMask;
  mProxy_->changePosition(effectorName, space, positionChange,
                          fractionMaxSpeed, axisMask);
  return true;
}

bool Cartesian_Control::getPosition(motion::getPosition::Request &req,
                                    motion::getPosition::Response &res) {

  string name = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.position = mProxy_->getPosition(name, space, useSensorValues);
  return true;
}

// bool Cartesian_Control::transformInterpolation(
//   motion::transformInterpolation::Request &req,
//   motion::transformInterpolation::Response &res) {
//   DEBUG("Service: transformInterpolation" << std::endl);

//   string name = req.name;
//   int space = req.space;
//   AL::ALValue path = req.path;
//   int axisMask = req.axisMask;
//   AL::ALValue duration = req.duration;
//   bool isAbsolute = req.isAbsolute;
//   mProxy_->transformInterpolation(name, space, path, axisMask, duration,
//                                   isAbsolute);
//   return true;
// }

// bool Cartesian_Control::transformInterpolations(
//   motion::transformInterpolations::Request &req,
//   motion::transformInterpolations::Response &res) {
//   DEBUG("Service: transformInterpolations" << std::endl);

//   std::vector<string> names = req.names;
//   int space = req.space;
//   size_t s = req.paths.size();
//   AL::ALValue paths;
//   AL::ALValue times;
//   paths.arraySetSize(s);
//   for (size_t i = 0; i < s; ++i) {
//     paths[i] = req.paths[i].floatList;
//     times[i] = req.times[i].floatList;
//   }
//   AL::ALValue axisMasks = req.axisMasks;
//   bool isAbsolute = req.isAbsolute;
//   mProxy_->transformInterpolations(names, space, paths, axisMasks, times,
//                                    isAbsolute);
//   return true;
// }

// bool Cartesian_Control::setTransform(motion::setTransform::Request &req,
//                                      motion::setTransform::Response &res) {
//   DEBUG("Service: setTransform" << std::endl);

//   string name = req.name;
//   int space = req.space;
//   std::vector<float> transform = req.transform;
//   float speed = req.speed;
//   int axisMask = req.axisMask;
//   mProxy_->setTransform(name, space, transform, speed, axisMask);
//   return true;
// }

// bool Cartesian_Control::changeTransform(motion::changeTransform::Request &req,
//                                         motion::changeTransform::Response &res) {
//   DEBUG("Service: changeTransform" << std::endl);

//   string name = req.name;
//   int space = req.space;
//   std::vector<float> transform = req.transform;
//   float speed = req.speed;
//   int axisMask = req.axisMask;
//   mProxy_->changeTransform(name, space, transform, speed, axisMask);
//   return true;
// }

bool Cartesian_Control::getTransform(motion::getTransform::Request &req,
                                     motion::getTransform::Response &res) {

  string nameJoint = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.transform = mProxy_->getTransform(nameJoint, space, useSensorValues);
  return true;
}
