#include <ros/ros.h>
#include "definitions.h"

#include "stiffness_control.h"
#include "joint_control.h"
#include "motion_task.h"

#include <alproxies/almotionproxy.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  Stiffness_Control StiffnessTest(&nh, &mProxy);
  Joint_Control JointTest(&nh, &mProxy);
  Motion_Task MotionTest(&nh, &mProxy);

  ros::Rate r(10);

  while (ros::ok())
  {
    StiffnessTest.spinTopics();
    ros::spinOnce();
    r.sleep();
  }
  
}
