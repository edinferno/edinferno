/* 
* @File: test.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:53:31
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-15 17:20:01
* @Desc: Test file to try out the Motion Wrapper functionality
*/

#include <ros/ros.h>
#include "motion_wrapper.h"
#include "definitions.h"

#include "motion/setStiffness.h"

int main(int argc, char *argv[]){

  
  Motion* motionTest = new Motion(argc, argv);

  ros::Rate r(10);
  motionTest->rest();

  // sleep(1);

  
  // // ros::init(argc, argv, "testNode");
  // ros::NodeHandle testNode;

  // ros::ServiceClient testClient = testNode.serviceClient<motion::setStiffness>("set_stiffness");
  // motion::setStiffness srv;
  // std::vector<string> test_names;
  // test_names.push_back("RArm");
  // srv.request.names = test_names;

  // std::vector<float> test_stiffness;
  // test_stiffness.push_back(1.0f);
  // srv.request.stiffnesses = test_stiffness;

  // if (testClient.call(srv))
  // {
  //   INFO("Stiffness set");
  //   // ROS_INFO("Stiffness set");
  // }
  // else
  // {
  //   ERR("Failed to call stiffness set!");
  //   // ROS_ERROR("Failed to call stiffness set!");
  // }

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  
  while (ros::ok())
  {
    DEBUG(".");
    motionTest->testSrv();
    motionTest->spinTopics();
    // ros::spinOnce();
    r.sleep();
  }
}