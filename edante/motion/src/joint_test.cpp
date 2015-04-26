/* 
* @Author: Alejandro Bordallo
* @Date:   2015-04-22 18:32:00
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-22 17:45:30
*/

#include <ros/ros.h>
#include "joint_control.h"
#include "definitions.h"

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "joint_control");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  Joint_Control JointTest(&nh, &mProxy);

  ros::Rate r(10);

  while (ros::ok())
  {
    // StiffnessTest->spinTopics();
    ros::spinOnce();
    r.sleep();
  }
}