
#ifndef LOCOMOTION_CONTROL_H_
#define LOCOMOTION_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <vector>
//#include <tuple>

#include <motion/getRobotPosition.h>
#include <motion/getNextRobotPosition.h>
#include <motion/getRobotVelocity.h>
#include <motion/getWalkArmsEnabled.h>
#include <motion/setWalkArmsEnabled.h>

#include <alproxies/almotionproxy.h>
#include "definitions.h"

using std::vector;

class Locomotion_Control{
public:
 Locomotion_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
 ~Locomotion_Control();
 
 // Locomotion Control API

 //void setWalkTargetVelocity(const float& x, const float& y, const float& theta, const float& frequency);
 //void setWalkTargetVelocity(const float& x, const float& y, const float& theta,
      //                         const float& frequency, vector<tuple<string, int> >& feetGaitConfig);
 //void setWalkTargetVelocity(const float& x, const float& y, const float& theta, 
 //                const float& frequency, vector<tuple<string, int> >& leftFootGaitConfig, vector<tuple<string, int> >& rightFootGaitConfig);

 //void move(const float& x, const float& y, const float& theta);
  //void move(const float& x, const float& y, const float& theta, vector<tuple<string, int> >& moveConfig);

 //void moveTo(const float& x, const float& y, const float& theta);
 //void moveTo(const float& x, const float& y, const float& theta, const vector<tuple<string, int> >& moveConfig);
 //void moveTo(const vector<tuple<float, float, float> >& controlPoints);
 //void moveTo(const vector<tuple<float, float, float> >& controlPoints,
                     //const vector<tuple<string, int> >& moveConfig);

 //void moveToward(const float& x, const float& y, const float& theta);
 //void moveToward(const float& x, const float& y, const float& theta,  vector<tuple<string, int> >& moveConfig);

 bool moveInit(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool waitUntilMoveIsFinished(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool moveIsActive();
 bool stopMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool getRobotPosition(motion::getRobotPosition::Request &req, motion::getRobotPosition::Response &res);
 bool getNextRobotPosition(motion::getNextRobotPosition::Request &req, motion::getNextRobotPosition::Response &res);
 bool getRobotVelocity(motion::getRobotVelocity::Request &req, motion::getRobotVelocity::Response &res);
 bool getWalkArmsEnabled(motion::getWalkArmsEnabled::Request &req, motion::getWalkArmsEnabled::Response &res);
 bool setWalkArmsEnabled(motion::setWalkArmsEnabled::Request &req, motion::setWalkArmsEnabled::Response &res);

 // ROS publisher
 void spinTopics();
private:
 // ROS
 ros::NodeHandle* nh_;
 ros::Publisher moving_pub_;
 ros::ServiceServer srv_moveInit_;
 ros::ServiceServer srv_waitMoveFinished_;
 ros::ServiceServer srv_stopMove_;
 ros::ServiceServer srv_getRobotPosition;
 ros::ServiceServer srv_getNextRobotPosition;
 ros::ServiceServer srv_getRobotVelocity;
 ros::ServiceServer srv_getWalkArmsEnabled;
 ros::ServiceServer srv_setWalkArmsEnabled;


 // NAOqi
 AL::ALMotionProxy* mProxy_;

};

#endif /* LOCOMOTION_CONTROL_H_ */