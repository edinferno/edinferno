/*
* @File: test.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:53:31
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-22 16:17:08
* @Desc: Test file to try out the Motion Wrapper functionality
*/

#include <ros/ros.h>
#include "stiffness_control.h"
#include "definitions.h"

#include "motion/setStiffness.h"

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "motion");
  Stiffness_Control* StiffnessTest = new Stiffness_Control();

  ros::Rate r(10);

  while (ros::ok())
  {
    StiffnessTest->spinTopics();
    ros::spinOnce();
    r.sleep();
  }
}
