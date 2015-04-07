/* 
* @File: test.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:53:31
* @Last Modified by:   Alejandro
* @Last Modified time: 2015-04-07 19:34:52
* @Desc: Test file to try out the Motion Wrapper functionality
*/

#include <ros/ros.h>
#include "motion_wrapper.h"
#include "definitions.h"

#include "motion/setStiffness.h"

int main(int argc, char *argv[]){
  ros::Rate r(10);
  ros::NodeHandle testNode;
  
  Motion* motionTest = new Motion(argc, argv);
  motionTest->wakeUp();

  ros::ServiceClient testClient = testNode.serviceClient<motion::setStiffness>("set_stiffness");
  motion::setStiffness srv;
  std::vector<string> test_names;
  test_names.push_back("RArm");
  srv.request.names = test_names;

  std::vector<float> test_stiffness;
  test_stiffness.push_back(0.0f);
  srv.request.stiffnesses = test_stiffness;

  if (testClient.call(srv))
  {
    ROS_INFO("Stiffness set");
  }
  else
  {
    ROS_ERROR("Failed to call stiffness set!");
  }

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}
