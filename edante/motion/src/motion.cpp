#include <ros/ros.h>
#include "definitions.h"

#include "stiffness_control.h"
#include "joint_control.h"
#include "locomotion_control.h"
#include "cartesian_control.h"
#include "body_balancer.h"
#include "fall_manager.h"
#include "motion_task.h"
#include "robot_posture.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  AL::ALRobotPostureProxy pProxy("127.0.0.1", 9559);
  AL::ALMemoryProxy memProxy("127.0.0.1", 9559);
  Stiffness_Control StiffnessTest(&nh, &mProxy);
  Joint_Control JointTest(&nh, &mProxy);
  Locomotion_Control LocomotionTest(&nh, &mProxy);
  Cartesian_Control CartesianTest(&nh, &mProxy);
  Body_Balancer BalancerTest(&nh, &mProxy);
  Fall_Manager FallManagerTest(&nh, &mProxy, &memProxy);
  Motion_Task MotionTest(&nh, &mProxy);
  Robot_Posture PostureTest(&nh, &pProxy);

  ros::Rate r(20);

  while (ros::ok()) {
    StiffnessTest.spinTopics();
    LocomotionTest.spinTopics();
    BalancerTest.spinTopics();
    FallManagerTest.spinTopics();
    ros::spinOnce();
    r.sleep();
  }

}
