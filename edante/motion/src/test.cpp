/* 
* @File: test.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:53:31
* @Last Modified by:   ubuntubeast
* @Last Modified time: 2015-04-07 16:38:26
* @Desc: Test file to try out the Motion Wrapper functionality
*/

#include <ros/ros.h>
#include "motion_wrapper.h"
#include "definitions.h"

int main(int argc, char *argv[]){
  ros::Rate r(10);
  Motion* motionTest = new Motion(argc, argv);
  motionTest->wakeUp();
  
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}
