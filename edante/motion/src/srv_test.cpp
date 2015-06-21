/**
 * @file      srv_test.cpp
 * @brief     Debug test file for testing complex services
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-19
 * @copyright (MIT) 2015 Edinferno
 */

#include <ros/ros.h>
#include "motion/motion_values.hpp"
#include <std_srvs/Empty.h>

#include <motion_msgs/PositionInterpolation.h>
#include <motion_msgs/GoToBalance.h>
#include <motion_msgs/SetPosture.h>
#include <motion_msgs/FootState.h>
#include <motion_msgs/Enable.h>
#include <motion_msgs/SetPosture.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "srv_test");

  ros::NodeHandle n;
  ros::Rate r(10);

  // KICK TEST
  float balance_init = 1.5f;
  float kick_time = 0.3f;
  float move_time = 0.4f;
  float lift_time = 0.5f;
  float leg_lift = 0.05f;
  float foot_retraction = 0.15f;
  float foot_forward_kick = 0.12f;
  float foot_lift = 0.06f;
  float foot_rot_kick = 0.6f;
  float balance_end = 0.7f;

  // Clients
  ros::ServiceClient move_init_client =
    n.serviceClient<std_srvs::Empty>("/motion/move_init");
  ros::ServiceClient balance_client =
    n.serviceClient<motion_msgs::Enable>("/motion/enable_balance");
  ros::ServiceClient pos_interp_client =
    n.serviceClient<motion_msgs::PositionInterpolation>("/motion/position_interpolation");
  ros::ServiceClient foot_client =
    n.serviceClient<motion_msgs::FootState>("/motion/foot_state");
  ros::ServiceClient goto_balance_client =
    n.serviceClient<motion_msgs::GoToBalance>("/motion/goto_balance");


  // Enable Balance
  motion_msgs::Enable balance_enable;
  balance_enable.request.is_enabled = true;

  if (balance_client.call(balance_enable)) {
    ROS_INFO("BalanceSrv Worked!");
  } else {
    ROS_INFO("Failed to call BalanceSrv service");
  }

  // Set foot states
  motion_msgs::FootState left_foot;
  left_foot.request.state_name = "Fixed";
  left_foot.request.support_leg = "RLeg";

  if (foot_client.call(left_foot)) {
    ROS_INFO("FootSrv1 Worked!");
  } else {
    ROS_INFO("Failed to call FootSrv1 service");
  }

  motion_msgs::FootState right_foot;
  right_foot.request.state_name = "Fixed";
  right_foot.request.support_leg = "LLeg";

  if (foot_client.call(right_foot)) {
    ROS_INFO("FootSrv1 Worked!");
  } else {
    ROS_INFO("Failed to call FootSrv1 service");
  }

  // Shift weight to Right leg
  motion_msgs::GoToBalance goto_balance_srv;
  goto_balance_srv.request.support_leg = "RLeg";
  goto_balance_srv.request.duration = balance_init;

  if (goto_balance_client.call(goto_balance_srv)) {
    ROS_INFO("GotoBalance Worked!");
  } else {
    ROS_INFO("Failed to call GotoBalance service");
  }

  // Disable Balance
  motion_msgs::Enable disable_balance;
  disable_balance.request.is_enabled = false;

  if (balance_client.call(disable_balance)) {
    ROS_INFO("BalanceSrv Worked!");
  } else {
    ROS_INFO("Failed to call BalanceSrv service");
  }

  // Lift Leg
  motion_msgs::PositionInterpolation lift_leg;
  lift_leg.request.chain_name = "LLeg";
  lift_leg.request.space = FRAME_ROBOT;
  lift_leg.request.path.traj_points.resize(1);
  lift_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg.request.path.traj_points[0].float_list.push_back(leg_lift);
  lift_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  lift_leg.request.axis_mask = 63;
  lift_leg.request.durations.resize(1);
  lift_leg.request.durations[0] = lift_time;
  lift_leg.request.is_absolute = false;

  if (pos_interp_client.call(lift_leg)) {
    ROS_INFO("Lift Worked!");
  } else {
    ROS_INFO("Failed to call Lift service");
  }

  // Move foot back
  motion_msgs::PositionInterpolation foot_back;
  foot_back.request.chain_name = "LLeg";
  foot_back.request.space = FRAME_ROBOT;
  foot_back.request.path.traj_points.resize(1);
  foot_back.request.path.traj_points[0].float_list.push_back(-foot_retraction);
  foot_back.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back.request.path.traj_points[0].float_list.push_back(leg_lift);
  foot_back.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back.request.path.traj_points[0].float_list.push_back(foot_rot_kick);
  foot_back.request.path.traj_points[0].float_list.push_back(0.0f);
  foot_back.request.axis_mask = 63;
  foot_back.request.durations.resize(1);
  foot_back.request.durations[0] = move_time;
  foot_back.request.is_absolute = false;

  if (pos_interp_client.call(foot_back)) {
    ROS_INFO("Back Worked!");
  } else {
    ROS_INFO("Failed to call Back service");
  }

  // Kick Forwards
  motion_msgs::PositionInterpolation kick_forw;
  kick_forw.request.chain_name = "LLeg";
  kick_forw.request.space = FRAME_ROBOT;
  kick_forw.request.path.traj_points.resize(1);
  kick_forw.request.path.traj_points[0].float_list.push_back(
    foot_retraction + foot_forward_kick);
  kick_forw.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw.request.path.traj_points[0].float_list.push_back(foot_lift);
  kick_forw.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw.request.path.traj_points[0].float_list.push_back(-foot_rot_kick);
  kick_forw.request.path.traj_points[0].float_list.push_back(0.0f);
  kick_forw.request.axis_mask = 63;
  kick_forw.request.durations.resize(1);
  kick_forw.request.durations[0] = kick_time;
  kick_forw.request.is_absolute = false;

  if (pos_interp_client.call(kick_forw)) {
    ROS_INFO("Forward Worked!");
  } else {
    ROS_INFO("Failed to call Forward service");
  }

  // Retract leg
  motion_msgs::PositionInterpolation retract_leg;
  retract_leg.request.chain_name = "LLeg";
  retract_leg.request.space = FRAME_ROBOT;
  retract_leg.request.path.traj_points.resize(1);
  retract_leg.request.path.traj_points[0].float_list.push_back(
    -foot_forward_kick);
  retract_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_leg.request.path.traj_points[0].float_list.push_back(0.0f);
  retract_leg.request.axis_mask = 63;
  retract_leg.request.durations.resize(1);
  retract_leg.request.durations[0] = move_time;
  retract_leg.request.is_absolute = false;

  if (pos_interp_client.call(retract_leg)) {
    ROS_INFO("Retract Worked!");
  } else {
    ROS_INFO("Failed to call Retract service");
  }

  // Put foot down
  motion_msgs::PositionInterpolation lower_foot;
  lower_foot.request.chain_name = "LLeg";
  lower_foot.request.space = FRAME_ROBOT;
  lower_foot.request.path.traj_points.resize(1);
  lower_foot.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_foot.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_foot.request.path.traj_points[0].float_list.push_back(-foot_lift);
  lower_foot.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_foot.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_foot.request.path.traj_points[0].float_list.push_back(0.0f);
  lower_foot.request.axis_mask = 63;
  lower_foot.request.durations.resize(1);
  lower_foot.request.durations[0] = move_time;
  lower_foot.request.is_absolute = false;

  if (pos_interp_client.call(lower_foot)) {
    ROS_INFO("Down Worked!");
  } else {
    ROS_INFO("Failed to call Down service");
  }

  if (balance_client.call(balance_enable)) {
    ROS_INFO("BalanceSrv Worked!");
  } else {
    ROS_INFO("Failed to call BalanceSrv service");
  }

  if (foot_client.call(left_foot)) {
    ROS_INFO("FootSrv1 Worked!");
  } else {
    ROS_INFO("Failed to call FootSrv1 service");
  }

  if (foot_client.call(right_foot)) {
    ROS_INFO("FootSrv1 Worked!");
  } else {
    ROS_INFO("Failed to call FootSrv1 service");
  }

  // Shift weight to Both legs
  motion_msgs::GoToBalance balance_final;
  balance_final.request.support_leg = "Legs";
  balance_final.request.duration = balance_end;

  if (goto_balance_client.call(balance_final)) {
    ROS_INFO("GotoBalance2 Worked!");
  } else {
    ROS_INFO("Failed to call GotoBalance2 service");
  }

  // Disable Balance
  if (balance_client.call(disable_balance)) {
    ROS_INFO("BalanceSrv Worked!");
  } else {
    ROS_INFO("Failed to call BalanceSrv service");
  }

  return 0;
}
