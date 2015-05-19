/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Main motion node
*/

#include <ros/ros.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>

#include "stiffness_control.h"
#include "joint_control.h"
#include "locomotion_control.h"
#include "cartesian_control.h"
#include "body_balancer.h"
#include "fall_manager.h"
#include "motion_task.h"
#include "robot_posture.h"
#include "definitions.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  AL::ALRobotPostureProxy pProxy("127.0.0.1", 9559);
  AL::ALMemoryProxy memProxy("127.0.0.1", 9559);
  StiffnessControl StiffnessTest(&nh, &mProxy);
  JointControl JointTest(&nh, &mProxy);
  LocomotionControl LocomotionTest(&nh, &mProxy);
  CartesianControl CartesianTest(&nh, &mProxy);
  BodyBalancer BalancerTest(&nh, &mProxy);
  FallManager FallManagerTest(&nh, &mProxy, &memProxy);
  MotionTask MotionTest(&nh, &mProxy);
  RobotPosture PostureTest(&nh, &pProxy);

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
