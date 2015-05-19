/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Debug test file for testing complex services
*/

#include <ros/ros.h>
// #include "joint_control.h"
#include "definitions.h"
#include "motion_values.h"
// #include <std_srvs/Empty.h>

// #include <ctime>

// #include "motion/SetStiffness.h"
// #include "motion/StiffnessInterp.h"
#include "motion/AngleInterp.h"
// #include "motion/SetAngles.h"
// #include "motion/GetTaskList.h"
// #include "motion/MoveTo.h"
// #include "motion/Move.h"
// #include "motion/MoveToward.h"
// #include "motion/GetAngles.h"
// #include "motion/PositionInterpolation.h"
// #include "motion/SetPosition.h"
// #include "motion/GetPosition.h"
// #include "motion/GetTransform.h"
// #include "motion/GoToBalance.h"
// #include "motion/SetEffectorControl.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "srv_test");

  ros::NodeHandle n;
  ros::Rate r(10);

  // // ANGLE INTERPOLATION TEST
  // ros::ServiceClient client1 =
  // n.serviceClient<motion::AngleInterp>("/motion/angleInterp");
  // motion::AngleInterp srv;

  // motion::AngleInterp serv;
  // serv.request.angle_lists.resize(2);
  // serv.request.time_lists.resize(2);
  // serv.request.names.push_back("HeadYaw");
  // serv.request.names.push_back("HeadPitch");
  // serv.request.angle_lists[0].float_list.push_back(1.0f);
  // serv.request.time_lists[0].float_list.push_back(1.0f);
  // serv.request.angle_lists[0].float_list.push_back(-1.0f);
  // serv.request.time_lists[0].float_list.push_back(5.0f);

  // serv.request.angle_lists[1].float_list.push_back(0.5f);
  // serv.request.time_lists[1].float_list.push_back(1.0f);
  // serv.request.angle_lists[1].float_list.push_back(0.0f);
  // serv.request.time_lists[1].float_list.push_back(5.0f);

  // srv.request.names = serv.request.names;
  // srv.request.angle_lists = serv.request.angle_lists;
  // srv.request.time_lists = serv.request.time_lists;
  // srv.request.is_absolute = true;

  // if (client1.call(srv)){
  //   DEBUG("angleInterpolation Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call angleInterpolation service" << std::endl);
  // }

  // // STIFFNESS INTERPOLATION TEST
  // ros::ServiceClient client1 =
  // n.serviceClient<motion::StiffnessInterp>("/motion/stiffnessInterpolation");
  // motion::StiffnessInterp srv;

  // motion::StiffnessInterp serv;
  // serv.request.stiffness_lists.resize(2);
  // serv.request.time_lists.resize(2);
  // serv.request.names.push_back("HeadYaw");
  // serv.request.names.push_back("HeadPitch");
  // serv.request.stiffness_lists[0].float_list.push_back(1.0f);
  // serv.request.time_lists[0].float_list.push_back(1.0f);
  // serv.request.stiffness_lists[0].float_list.push_back(0.0f);
  // serv.request.time_lists[0].float_list.push_back(5.0f);

  // serv.request.stiffness_lists[1].float_list.push_back(0.0f);
  // serv.request.time_lists[1].float_list.push_back(1.0f);
  // serv.request.stiffness_lists[1].float_list.push_back(1.0f);
  // serv.request.time_lists[1].float_list.push_back(5.0f);

  // srv.request.names = serv.request.names;
  // srv.request.stiffness_lists = serv.request.stiffness_lists;
  // srv.request.time_lists = serv.request.time_lists;

  // if (client1.call(srv)){
  //   DEBUG("angleInterpolation Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call angleInterpolation service" << std::endl);
  // }

  // // GET TASK LIST TEST
  // sleep(1);

  // ros::ServiceClient client2 =
  // n.serviceClient<motion::GetTaskList>("motion/getTaskList");
  // motion::GetTaskList srv2;

  // if (client2.call(srv2)){
  //   DEBUG("getTaskList Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call getTaskList service" << std::endl);
  // }

  // DEBUG("TaskList:")
  // DEBUG(srv2.response.task_list[0].task_name);
  // DEBUG(srv2.response.task_list[0].motion_id);
  // DEBUG(std::endl);

  // // KILL TASK TEST
  // sleep(1);

  // ros::ServiceClient client3 =
  // n.serviceClient<std_srvs::Empty>("motion/killAll");
  // std_srvs::Empty srv3;

  // if (client3.call(srv3)){
  //   DEBUG("KillAll Worked!" << std::endl);
  // }
  // else{
  //   ERR("Failed to call KillAll service" << std::endl);
  // }

  // ANGLE INTERPOLATION TEST
  // ros::ServiceClient client1 =
  //   n.serviceClient<motion::setEffectorControl>("/motion/setEffectorControl",
  //       true);
  // motion::setEffectorControl srv1;
  // ros::ServiceClient client2 =
  // n.serviceClient<motion::PositionInterpolation>
  // ("/motion/positionInterpolation", true);
  // motion::PositionInterpolation srv2;

  // motion::PositionInterpolation serv;
  // serv.request.target_velocity.resize(1);
  // serv.request.norm_velocity.x = 0.2f;
  // serv.request.norm_velocity.y = 0.0f;
  // serv.request.norm_velocity.theta = 0.0f;

  // serv.request.move_configuration.names.resize(1);
  // serv.request.move_configuration.names[0] = "MaxStepX";
  // serv.request.move_configuration.values.resize(1);
  // serv.request.move_configuration.values[0] = 0.001f;

  // serv.request.control_points[1].x = 0.5f;
  // serv.request.control_points[1].y = 0.0f;
  // serv.request.control_points[1].theta = 0.0f;

  // serv.request.control_points[2].x = 0.0f;
  // serv.request.control_points[2].y = 0.0f;
  // serv.request.control_points[2].theta = -3.14f;

  // serv.request.control_points[3].x = 0.5f;
  // serv.request.control_points[3].y = 0.0f;
  // serv.request.control_points[3].theta = 0.0f;

  // serv.request.control_points[4].x = 0.0f;
  // serv.request.control_points[4].y = 0.0f;
  // serv.request.control_points[4].theta = -3.14f;

  // serv.request.move_configuration;

  // srv.request.norm_velocity = serv.request.norm_velocity;
  // srv.request.move_configuration = serv.request.move_configuration;

  // srv1.request.chain_name = "Torso";
  // srv1.request.space = FRAME_ROBOT;
  // srv1.request.path.traj_points.resize(1);
  // srv1.request.path.traj_points[0].float_list.push_back(0.0f);
  // srv1.request.path.traj_points[0].float_list.push_back(-0.07f);
  // srv1.request.path.traj_points[0].float_list.push_back(-0.03f);
  // srv1.request.path.traj_points[0].float_list.push_back(0.0f);
  // srv1.request.path.traj_points[0].float_list.push_back(0.0f);
  // srv1.request.path.traj_points[0].float_list.push_back(0.0f);
  // srv1.request.axis_mask = 63;
  // srv1.request.durations.resize(1);
  // srv1.request.durations[0] = 2.0f;
  // srv1.request.is_absolute = false;

  // if (client1.call(srv1)) {
  //   DEBUG("positionInterp1 Worked!" << std::endl);
  // } else {
  //   ERR("Failed to call positionInterp1 service" << std::endl);
  // }

  // srv1.request.chain_name = "Torso";
  // srv1.request.space = FRAME_ROBOT;
  // std::vector<float> position(6, 0.0f); // Absolute Position
  // position[2] = 0.25f;
  // srv1.request.position = position;
  // srv1.request.fraction_max_speed = 0.2f;
  // srv1.request.axis_mask = 63;

  // if (client1.call(srv1)) {
  //   DEBUG("setPosition Worked!" << std::endl);
  // } else {
  //   ERR("Failed to call setPosition service" << std::endl);
  // }


  // REQUIRES BALANCING!!

  //   # Activate Whole Body Balancer.
  // is_enabled  = True
  // proxy.wbEnable(is_enabled)

  // # Legs are constrained in a plane
  // state_name  = "Plane"
  // support_leg = "Legs"
  // proxy.wbFootState(state_name, support_leg)

  // ANGLE INTERPOLATION TEST
  ros::ServiceClient client1 =
    n.serviceClient<motion::AngleInterp>("/motion/angle_interp");
  motion::AngleInterp srv;

  srv.request.angle_lists.resize(1);
  srv.request.time_lists.resize(1);
  srv.request.names.push_back("LHipYawPitch");
  srv.request.angle_lists[0].float_list.push_back((-45.0f * PI) / 180);
  srv.request.time_lists[0].float_list.push_back(3.0f);
  srv.request.angle_lists[0].float_list.push_back((10.0f * PI) / 180);
  srv.request.time_lists[0].float_list.push_back(6.0f);
  srv.request.angle_lists[0].float_list.push_back((0.0f * PI) / 180);
  srv.request.time_lists[0].float_list.push_back(9.0f);

  srv.request.is_absolute = true;

  if (client1.call(srv)) {
    DEBUG("angleInterpolation Worked!" << std::endl);
  } else {
    ERR("Failed to call angleInterpolation service" << std::endl);
  }

// srv1.request.name = "RArm";
// srv1.request.space = FRAME_TORSO;
// srv1.request.use_sensor_values = false;

// if (client1.call(srv1)) {
//   DEBUG("getTransform Worked!" << std::endl);
//   DEBUG("Transform: ")
//   for (size_t i = 0; i < 16; ++i) {
//     DEBUG(srv1.response.transform[i] << ", ")
//   }
//   DEBUG(std::endl);
// } else {
//   ERR("Failed to call getTransform service" << std::endl);
// }

// srv1.request.effector_names.resize(2);
// srv1.request.effector_names[0] = "LArm";
// srv1.request.effector_names[1] = "RArm";
// srv1.request.space = FRAME_ROBOT;

// srv1.request.paths.resize(2);
// srv1.request.paths[0].traj_points.resize(1);
// srv1.request.paths[1].traj_points.resize(1);
// srv1.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[0].traj_points[0].float_list.push_back(-0.04f);
// srv1.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.04f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv1.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv1.request.axis_masks.resize(2);
// srv1.request.axis_masks[0] = AXIS_MASK_VEL;
// srv1.request.axis_masks[1] = AXIS_MASK_VEL;
// srv1.request.durations.resize(2);
// srv1.request.durations[0].float_list.push_back(1.0f);
// srv1.request.durations[1].float_list.push_back(1.0f);
// srv1.request.is_absolute = false;

// if (client1.call(srv1)) {
//   DEBUG("positionsInterp1 Worked!" << std::endl);
// } else {
//   ERR("Failed to call positionInterp1 service" << std::endl);
// }

// float dx = 0.03;
// float dy = 0.04;
// srv2.request.effector_names.resize(3);
// srv2.request.effector_names[0] = "LArm";
// srv2.request.effector_names[1] = "RArm";
// srv2.request.effector_names[2] = "Torso";
// srv2.request.space = FRAME_ROBOT;
// srv2.request.paths.resize(3);
// srv2.request.paths[0].traj_points.resize(1);
// srv2.request.paths[1].traj_points.resize(1);
// srv2.request.paths[2].traj_points.resize(4);
// srv2.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[0].traj_points[0].float_list.push_back(-0.04f);
// srv2.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[0].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.04f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[1].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[0].float_list.push_back(dy);
// srv2.request.paths[2].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[0].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[1].float_list.push_back(-dx);
// srv2.request.paths[2].traj_points[1].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[1].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[1].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[1].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[1].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[2].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[2].float_list.push_back(-dy);
// srv2.request.paths[2].traj_points[2].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[2].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[2].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[2].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.paths[2].traj_points[3].float_list.push_back(0.0f);
// srv2.request.axis_masks.resize(3);
// srv2.request.axis_masks[0] = AXIS_MASK_VEL;
// srv2.request.axis_masks[1] = AXIS_MASK_VEL;
// srv2.request.axis_masks[2] = AXIS_MASK_ALL;
// srv2.request.durations.resize(3);
// srv2.request.durations[0].float_list.push_back(1.0f);
// srv2.request.durations[1].float_list.push_back(1.0f);
// srv2.request.durations[2].float_list.push_back(1.0f);
// srv2.request.durations[2].float_list.push_back(2.0f);
// srv2.request.durations[2].float_list.push_back(3.0f);
// srv2.request.durations[2].float_list.push_back(4.0f);
// srv2.request.is_absolute = false;

// if (client2.call(srv2)) {
//   DEBUG("positionsInterp2 Worked!" << std::endl);
// } else {
//   ERR("Failed to call positionInterp2 service" << std::endl);
// }

// srv.request.names.resize(1);
// srv.request.names[0] = "RArm";
// srv.request.use_sensors = true;
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
