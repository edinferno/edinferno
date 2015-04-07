/* 
* @File: motion_wrapper.h
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-04 20:47:59
* @Desc: Defines the Motion Wrapper functions
*/

#ifndef MOTION_WRAPPER_H_
#define MOTION_WRAPPER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <alproxies/almotionproxy.h>

#include "definitions.h"

class Motion{
  public:
    Motion(int argc, char *argv[]);
    ~Motion();

    // Wrapper Functions
    void wakeUp();
    void rest();

  private:
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher wake_pub_;

    // NAOqi
    AL::ALMotionProxy* mProxy_;

    // Internal
    bool awake_;
};

#endif /* MOTION_WRAPPER_H_ */