#include <ros/ros.h>
// #include "joint_control.h"
#include "definitions.h"
#include <std_srvs/Empty.h>

// #include "motion/setStiffness.h"
#include "motion/angleInterp.h"
// #include "motion/stiffnessInterp.h"
#include "motion/setAngles.h"
#include "motion/getTaskList.h"

int main(int argc, char *argv[]){

  using namespace std;
  ros::init(argc, argv, "srv_test");

  ros::NodeHandle n;
  ros::Rate r(10);

  // ANGLE INTERPOLATION TEST
  ros::ServiceClient client1 = n.serviceClient<motion::angleInterp>("/motion/angleInterp");
  motion::angleInterp srv;

  motion::angleInterp serv;
  serv.request.angleLists.resize(2);
  serv.request.timeLists.resize(2);
  serv.request.names.push_back("HeadYaw");
  serv.request.names.push_back("HeadPitch");
  serv.request.angleLists[0].floatList.push_back(1.0f);
  serv.request.timeLists[0].floatList.push_back(1.0f);
  serv.request.angleLists[0].floatList.push_back(-1.0f);
  serv.request.timeLists[0].floatList.push_back(5.0f);

  serv.request.angleLists[1].floatList.push_back(0.5f);
  serv.request.timeLists[1].floatList.push_back(1.0f);
  serv.request.angleLists[1].floatList.push_back(0.0f);
  serv.request.timeLists[1].floatList.push_back(5.0f);

  srv.request.names = serv.request.names;
  srv.request.angleLists = serv.request.angleLists;
  srv.request.timeLists = serv.request.timeLists;
  srv.request.isAbsolute = true;

  if (client1.call(srv)){
    DEBUG("angleInterpolation Worked!" << std::endl);
  }
  else{
    ERR("Failed to call angleInterpolation service" << std::endl);
  }

  // // STIFFNESS INTERPOLATION TEST
  // ros::ServiceClient client1 = n.serviceClient<motion::stiffnessInterp>("/motion/stiffnessInterpolation");
  // motion::stiffnessInterp srv;

  // motion::stiffnessInterp serv;
  // serv.request.stiffnessLists.resize(2);
  // serv.request.timeLists.resize(2);
  // serv.request.names.push_back("HeadYaw");
  // serv.request.names.push_back("HeadPitch");
  // serv.request.stiffnessLists[0].floatList.push_back(1.0f);
  // serv.request.timeLists[0].floatList.push_back(1.0f);
  // serv.request.stiffnessLists[0].floatList.push_back(0.0f);
  // serv.request.timeLists[0].floatList.push_back(5.0f);

  // serv.request.stiffnessLists[1].floatList.push_back(0.0f);
  // serv.request.timeLists[1].floatList.push_back(1.0f);
  // serv.request.stiffnessLists[1].floatList.push_back(1.0f);
  // serv.request.timeLists[1].floatList.push_back(5.0f);

  // srv.request.names = serv.request.names;
  // srv.request.stiffnessLists = serv.request.stiffnessLists;
  // srv.request.timeLists = serv.request.timeLists;

  // if (client1.call(srv)){
  //   DEBUG("angleInterpolation Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call angleInterpolation service" << std::endl);
  // }

  // GET TASK LIST TEST
  sleep(1);

  ros::ServiceClient client2 = n.serviceClient<motion::getTaskList>("motion/getTaskList");
  motion::getTaskList srv2;

  if (client2.call(srv2)){
    DEBUG("getTaskList Worked!" << std::endl);
  }
  else{
    ERR("Failed to call getTaskList service" << std::endl);
  }

  DEBUG("TaskList:")
  DEBUG(srv2.response.taskList[0].taskName);
  DEBUG(srv2.response.taskList[0].motionID);
  DEBUG(std::endl);

  // KILL TASK TEST
  sleep(1);

  ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("motion/killAll");
  std_srvs::Empty srv3;

  if (client3.call(srv3)){
    DEBUG("KillAll Worked!" << std::endl);
  }
  else{
    ERR("Failed to call KillAll service" << std::endl);
  }

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}