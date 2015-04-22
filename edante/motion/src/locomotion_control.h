
#ifndef LOCOMOTION_CONTROL_H_
#define LOCOMOTION_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>
//#include <tuple>

#include <alproxies/almotionproxy.h>

#include "definitions.h"

using namespace std;

class Locomotion_Control{
  public:
    Locomotion_Control();
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

    void moveInit();
    //void waitUntilMoveIsFinished();
    //bool moveIsActive();
    //void stopMove();
    //vector<float> getRobotPosition(const bool& useSensors);
    //vector<float> getNextRobotPosition();
    //vector<float> getRobotVelocity();
    //vector<bool> getWalkArmsEnabled();
    //void setWalkArmsEnabled(const bool& leftArmEnable, const bool& rightArmEnable);

    // ROS publisher
    void spinTopics();
  private:
    // ROS
    ros::NodeHandle* nh_;
    ros::Publisher moving_pub_;
    // NAOqi
    AL::ALMotionProxy* mProxy_;

    bool moving_;

};

#endif /* LOCOMOTION_CONTROL_H_ */