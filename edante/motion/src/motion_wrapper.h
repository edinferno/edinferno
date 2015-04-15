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
#include <vector>

#include "motion/setStiffness.h"

#include <alproxies/almotionproxy.h>

#include "definitions.h"

using namespace std;

class Motion{
  public:
    Motion(int argc, char *argv[]);
    ~Motion();

    // Stiffness control API
    void wakeUp();
    void rest();
    void setStiffnesses(const vector<string>& names, 
                        const vector<float>& stiffnesses);
    vector<float> getStiffnesses(const vector<string>&);

    // ROS publisher
    void spinTopics();

    // ROS services
    bool recStiffness(motion::setStiffness::Request &req,
                      motion::setStiffness::Response &res);
    void testSrv();

  private:
    // ROS
    ros::NodeHandle* nh_;
    ros::Publisher wake_pub_;
    ros::ServiceServer set_stiffness_;

    // NAOqi
    AL::ALMotionProxy* mProxy_;

    // Internal
    bool awake_;
};

#endif /* MOTION_WRAPPER_H_ */