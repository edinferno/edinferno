/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's audio class
*/

#include "audio.h"

Audio::Audio(ros::NodeHandle* nh) {
  nh_ = nh;
  audio = new AL::ALAudioDeviceProxy("127.0.0.1", 9559);
  INFO("Setting up Audio signalling services" << std::endl);
  srv_play_sine_ = nh_->advertiseService("playSine",
                                         &Audio::playSine, this);
}

Audio::~Audio() {
}

bool Audio::playSine(signalling::playSine::Request &req,
                     signalling::playSine::Response &res) {
  audio->playSine(req.frequence, req.gain, req.pan, req.duration);
  return true;
}
