#include <ros/ros.h>
#include "definitions.h"

#include "motion_task.h"

#include <alproxies/almotionproxy.h>

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "motion_task");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  Motion_Task MotionTest(&nh, &mProxy);

  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
}
