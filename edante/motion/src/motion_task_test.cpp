#include <ros/ros.h>
#include "motion_task.h"
#include "definitions.h"

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "motion_task");
  Motion_Task* MotionTest = new Motion_Task();

  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
}
