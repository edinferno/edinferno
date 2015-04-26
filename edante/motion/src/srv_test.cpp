#include <ros/ros.h>
// #include "joint_control.h"
#include "definitions.h"

// #include "motion/setStiffness.h"
// #include "motion/angleInterp.h"
#include "motion/stiffnessInterp.h"
#include "motion/float32List.h"

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "srv_test");

  ros::NodeHandle n;
  ros::Rate r(10);

  // ros::ServiceClient client = n.serviceClient<motion::angleInterp>("/motion/angleInterp");
  // motion::angleInterp srv;

  // motion::angleInterp serv;
  // serv.request.angleLists.resize(2);
  // serv.request.timeLists.resize(2);
  // serv.request.names.push_back("HeadYaw");
  // serv.request.names.push_back("HeadPitch");
  // serv.request.angleLists[0].floatList.push_back(1.0f);
  // serv.request.timeLists[0].floatList.push_back(1.0f);
  // serv.request.angleLists[0].floatList.push_back(-1.0f);
  // serv.request.timeLists[0].floatList.push_back(2.0f);

  // serv.request.angleLists[1].floatList.push_back(0.5f);
  // serv.request.timeLists[1].floatList.push_back(1.0f);
  // serv.request.angleLists[1].floatList.push_back(0.0f);
  // serv.request.timeLists[1].floatList.push_back(1.5f);

  // srv.request.names = serv.request.names;
  // srv.request.angleLists = serv.request.angleLists;
  // srv.request.timeLists = serv.request.timeLists;
  // srv.request.isAbsolute = true;

  ros::ServiceClient client = n.serviceClient<motion::stiffnessInterp>("/motion/stiffnessInterpolation");
  motion::stiffnessInterp srv;

  motion::stiffnessInterp serv;
  serv.request.stiffnessLists.resize(2);
  serv.request.timeLists.resize(2);
  serv.request.names.push_back("HeadYaw");
  serv.request.names.push_back("HeadPitch");
  serv.request.stiffnessLists[0].floatList.push_back(1.0f);
  serv.request.timeLists[0].floatList.push_back(1.0f);
  serv.request.stiffnessLists[0].floatList.push_back(0.0f);
  serv.request.timeLists[0].floatList.push_back(5.0f);

  serv.request.stiffnessLists[1].floatList.push_back(0.0f);
  serv.request.timeLists[1].floatList.push_back(1.0f);
  serv.request.stiffnessLists[1].floatList.push_back(1.0f);
  serv.request.timeLists[1].floatList.push_back(5.0f);

  srv.request.names = serv.request.names;
  srv.request.stiffnessLists = serv.request.stiffnessLists;
  srv.request.timeLists = serv.request.timeLists;

  if (client.call(srv))
  {
    DEBUG("Worked!");
  }
  else
  {
    ERR("Failed to call service");
  }

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}