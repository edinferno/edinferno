/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Add file description...
*/

#include <ros/ros.h>
#include "signalling/playSine.h"

#include <alproxies/alaudiodeviceproxy.h>

#include "definitions.h"

#ifndef AUDIO_H_
#define AUDIO_H_

class Audio {
 public:
  Audio(ros::NodeHandle* nh);
  ~Audio();

  bool playSine(signalling::playSine::Request &req,
                signalling::playSine::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_play_sine_;

  // NaoQI
  AL::ALAudioDeviceProxy* audio;

};

#endif  /* AUDIO_H_ */