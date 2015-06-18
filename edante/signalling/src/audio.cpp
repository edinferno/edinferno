/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's audio class
*/

#include "signalling/audio.hpp"

Audio::Audio(ros::NodeHandle* nh, std::string naoqi_ip, int naoqi_port) {
  nh_ = nh;
  audio_ = new AL::ALAudioDeviceProxy(naoqi_ip, naoqi_port);
  speech_ = new AL::ALTextToSpeechProxy(naoqi_ip, naoqi_port);
  ROS_INFO_STREAM("Setting up Audio signalling services");
  srv_play_sine_ = nh_->advertiseService("play_sine", &Audio::playSine, this);
  srv_say_text_ = nh_->advertiseService("say_text", &Audio::sayText, this);
}

Audio::~Audio() {
}

bool Audio::playSine(signalling_msgs::PlaySine::Request& req,
                     signalling_msgs::PlaySine::Response& res) {
  audio_->playSine(req.frequency, req.gain, req.pan, req.duration);
  return true;
}

bool Audio::sayText(signalling_msgs::SayText::Request& req,
                    signalling_msgs::SayText::Response& res) {
  speech_->say(req.text);
  return true;
}
