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

  using namespace std;
  ros::init(argc, argv, "motion");
  Motion* motionTest = new Motion();

  ros::Rate r(10);

  while (ros::ok())
  {
    motionTest->spinTopics();
    ros::spinOnce();
    r.sleep();
  }
}
