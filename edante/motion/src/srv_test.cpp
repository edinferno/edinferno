#include <ros/ros.h>
// #include "joint_control.h"
#include "definitions.h"
#include <std_srvs/Empty.h>

#include <ctime>

// #include "motion/setStiffness.h"
// #include "motion/stiffnessInterp.h"
// #include "motion/angleInterp.h"
// #include "motion/setAngles.h"
// #include "motion/getTaskList.h"
// #include "motion/moveTo.h"
// #include "motion/move.h"
// #include "motion/moveToward.h"
// #include "motion/getAngles.h"
#include "motion/positionInterpolation.h"

int main(int argc, char *argv[]) {

  using namespace std;
  ros::init(argc, argv, "srv_test");

  ros::NodeHandle n;
  ros::Rate r(10);

  // // ANGLE INTERPOLATION TEST
  // ros::ServiceClient client1 = n.serviceClient<motion::angleInterp>("/motion/angleInterp");
  // motion::angleInterp srv;

  // motion::angleInterp serv;
  // serv.request.angleLists.resize(2);
  // serv.request.timeLists.resize(2);
  // serv.request.names.push_back("HeadYaw");
  // serv.request.names.push_back("HeadPitch");
  // serv.request.angleLists[0].floatList.push_back(1.0f);
  // serv.request.timeLists[0].floatList.push_back(1.0f);
  // serv.request.angleLists[0].floatList.push_back(-1.0f);
  // serv.request.timeLists[0].floatList.push_back(5.0f);

  // serv.request.angleLists[1].floatList.push_back(0.5f);
  // serv.request.timeLists[1].floatList.push_back(1.0f);
  // serv.request.angleLists[1].floatList.push_back(0.0f);
  // serv.request.timeLists[1].floatList.push_back(5.0f);

  // srv.request.names = serv.request.names;
  // srv.request.angleLists = serv.request.angleLists;
  // srv.request.timeLists = serv.request.timeLists;
  // srv.request.isAbsolute = true;

  // if (client1.call(srv)){
  //   DEBUG("angleInterpolation Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call angleInterpolation service" << std::endl);
  // }

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

  // // GET TASK LIST TEST
  // sleep(1);

  // ros::ServiceClient client2 = n.serviceClient<motion::getTaskList>("motion/getTaskList");
  // motion::getTaskList srv2;

  // if (client2.call(srv2)){
  //   DEBUG("getTaskList Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call getTaskList service" << std::endl);
  // }

  // DEBUG("TaskList:")
  // DEBUG(srv2.response.taskList[0].taskName);
  // DEBUG(srv2.response.taskList[0].motionID);
  // DEBUG(std::endl);

  // // KILL TASK TEST
  // sleep(1);

  // ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("motion/killAll");
  // std_srvs::Empty srv3;

  // if (client3.call(srv3)){
  //   DEBUG("KillAll Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call KillAll service" << std::endl);
  // }

  // ANGLE INTERPOLATION TEST
  ros::ServiceClient client1 =
    n.serviceClient<motion::positionInterpolation>("/motion/positionInterpolation",
        true);
  motion::positionInterpolation srv1;
  ros::ServiceClient client2 =
    n.serviceClient<motion::positionInterpolation>("/motion/positionInterpolation",
        true);
  motion::positionInterpolation srv2;

  // motion::positionInterpolation serv;
  // serv.request.targetVelocity.resize(1);
  // serv.request.normVelocity.x = 0.2f;
  // serv.request.normVelocity.y = 0.0f;
  // serv.request.normVelocity.theta = 0.0f;

  // serv.request.moveConfiguration.names.resize(1);
  // serv.request.moveConfiguration.names[0] = "MaxStepX";
  // serv.request.moveConfiguration.values.resize(1);
  // serv.request.moveConfiguration.values[0] = 0.001f;

  // serv.request.controlPoints[1].x = 0.5f;
  // serv.request.controlPoints[1].y = 0.0f;
  // serv.request.controlPoints[1].theta = 0.0f;

  // serv.request.controlPoints[2].x = 0.0f;
  // serv.request.controlPoints[2].y = 0.0f;
  // serv.request.controlPoints[2].theta = -3.14f;

  // serv.request.controlPoints[3].x = 0.5f;
  // serv.request.controlPoints[3].y = 0.0f;
  // serv.request.controlPoints[3].theta = 0.0f;

  // serv.request.controlPoints[4].x = 0.0f;
  // serv.request.controlPoints[4].y = 0.0f;
  // serv.request.controlPoints[4].theta = -3.14f;

  // serv.request.moveConfiguration;

  // srv.request.normVelocity = serv.request.normVelocity;
  // srv.request.moveConfiguration = serv.request.moveConfiguration;

  srv1.request.chainName = "Torso";
  srv1.request.space = 2;
  srv1.request.path.resize(6);
  srv1.request.path[0] = 0.0f;
  srv1.request.path[1] = -0.07f;
  srv1.request.path[2] = -0.03f;
  srv1.request.path[3] = 0.0f;
  srv1.request.path[4] = 0.0f;
  srv1.request.path[5] = 0.0f;
  srv1.request.axisMask = 63;
  srv1.request.durations.resize(1);
  srv1.request.durations[0] = 2.0f;
  srv1.request.isAbsolute = false;

  if (client1.call(srv1)) {
    DEBUG("positionInterp1 Worked!" << std::endl);
  } else {
    ERR("Failed to call positionInterp1 service" << std::endl);
  }

  srv2.request.chainName = "LLeg";
  srv2.request.space = 2;
  srv2.request.path.resize(6);
  srv2.request.path[0] = 0.0f;
  srv2.request.path[1] = 0.06f;
  srv2.request.path[2] = 0.00f;
  srv2.request.path[3] = 0.0f;
  srv2.request.path[4] = 0.0f;
  srv2.request.path[5] = 0.8f;
  srv2.request.axisMask = 63;
  srv2.request.durations.resize(1);
  srv2.request.durations[0] = 2.0f;
  srv2.request.isAbsolute = false;

  if (client2.call(srv2)) {
    DEBUG("positionInterp2 Worked!" << std::endl);
  } else {
    ERR("Failed to call positionInterp2 service" << std::endl);
  }

  // srv.request.names.resize(1);
  // srv.request.names[0] = "RArm";
  // srv.request.useSensors = true;
  // int nSucc;


  // clock_t begin = clock();

  // for (int i = 0; i < 100; ++i) {
  //   if (client1.call(srv)) {
  //     // DEBUG("moveToward Worked!" << std::endl);
  //     nSucc++;
  //   } else {
  //     // ERR("Failed to call moveToward service" << std::endl);
  //   }
  // }

  // clock_t end = clock();
  // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  // cout
  //     << "Succ: " << nSucc
  //     << " Dur: " << elapsed_secs
  //     << " Clock: " << CLOCKS_PER_SEC
  //     << std::endl;

  // while (ros::ok()) {
  //   ros::spinOnce();
  //   r.sleep();
  // }
  return 0;
}
