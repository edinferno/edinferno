/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's audio class
*/

#include <ros/ros.h>

#include <alproxies/alaudiodeviceproxy.h>

#include <signalling_msgs/PlaySine.h>

#ifndef AUDIO_HPP
#define AUDIO_HPP

class Audio {
 public:
  explicit Audio(ros::NodeHandle* nh);
  ~Audio();

  bool playSine(signalling_msgs::PlaySine::Request& req,
                signalling_msgs::PlaySine::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_play_sine_;

  // NaoQI
  AL::ALAudioDeviceProxy* audio_;
};

#endif  /* AUDIO_H_ */
