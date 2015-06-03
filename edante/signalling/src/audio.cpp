/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's audio class
*/

#include "signalling/audio.hpp"

Audio::Audio(ros::NodeHandle* nh) {
  nh_ = nh;
  audio_ = new AL::ALAudioDeviceProxy("127.0.0.1", 9559);
  ROS_INFO_STREAM("Setting up Audio signalling services");
  srv_play_sine_ = nh_->advertiseService("play_sine", &Audio::playSine, this);
}

Audio::~Audio() {
}

bool Audio::playSine(signalling_msgs::PlaySine::Request& req,
                     signalling_msgs::PlaySine::Response& res) {
  audio_->playSine(req.frequency, req.gain, req.pan, req.duration);
  return true;
}
