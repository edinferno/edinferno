/**
 * @file      joy_remote.cpp
 * @brief     Joystick remote control node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-22
 * @copyright (MIT) 2015 Edinferno
 */

#include "joy_remote.h"

JoyRemote::JoyRemote(ros::NodeHandle* nh) {
  nh_ = nh;
  joy_sub_ = nh_->subscribe("joy", 1000, &JoyRemote::joyCallback, this);
  moveTowardClient = nh_->serviceClient<motion::MoveToward>(
                       "/motion/move_toward", true);
  moveTowardClient.waitForExistence();
  wakeUpClient = nh_->serviceClient<std_srvs::Empty>(
                   "/motion/wake_up", true);
  wakeUpClient.waitForExistence();
  moveInitClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/move_init", true);
  moveInitClient.waitForExistence();
  standClient = nh_->serviceClient<motion::SetPosture>(
                  "/motion/goto_posture", true);
  standClient.waitForExistence();
  restClient = nh_->serviceClient<std_srvs::Empty>(
                 "/motion/rest", true);
  restClient.waitForExistence();
  stopMoveClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/stop_move", true);
  stopMoveClient.waitForExistence();
  changeAnglesClient = nh_->serviceClient<motion::ChangeAngles>(
                         "/motion/change_angles", true);
  changeAnglesClient.waitForExistence();
  INFO("Joystick remote node initialised" << std::endl);
  wakeUpFlag = false;
  moveInitFlag = false;
  standFlag = false;
  restFlag = false;
  standSrv.request.speed = 0.5f;
  headSrv.request.names.push_back("HeadYaw");
  headSrv.request.names.push_back("HeadPitch");
  headSrv.request.changes.resize(2);
  headSrv.request.fraction_max_speed = 0.1f;
}

JoyRemote::~JoyRemote() {
}

void JoyRemote::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->buttons[1]) {
    wakeUpFlag = true;
  } else if (msg->buttons[2]) {
    moveInitFlag = true;
  } else if (msg->buttons[3]) {
    standFlag = true;
  } else if (msg->buttons[0]) {
    restFlag = true;
  } else {
    float x = msg->axes[1];
    float y = msg->axes[0];
    float theta = msg->axes[3];
    moveSrv.request.norm_velocity.x = x;
    moveSrv.request.norm_velocity.y = y;
    moveSrv.request.norm_velocity.theta = theta;
    float headYawAngle = msg->axes[6];
    float headPitchAngle = msg->axes[7];
    headSrv.request.changes[0] = headYawAngle;
    headSrv.request.changes[1] = headPitchAngle * -1;
  }
}

void JoyRemote::sendCmd() {
  if (wakeUpFlag) {
    this->wakeUp();
  } else if (moveInitFlag) {
    this->moveInit();
  } else if (standFlag) {
    this->stand();
  } else if (restFlag) {
    this->rest();
  } else {
    moveTowardClient.call(moveSrv);
    changeAnglesClient.call(headSrv);
  }
}

void JoyRemote::wakeUp() {
  wakeUpClient.call(wakeUpSrv);
  wakeUpFlag = false;
}

void JoyRemote::moveInit() {
  moveInitClient.call(moveInitSrv);
  moveInitFlag = false;
}

void JoyRemote::stand() {
  wakeUpClient.call(wakeUpSrv);
  standSrv.request.posture_name = "Stand";
  standClient.call(standSrv);
  standFlag = false;
}

void JoyRemote::rest() {
  stopMoveClient.call(stopSrv);
  sleep(0.25);
  moveInitClient.call(moveInitSrv);
  sleep(0.25);
  restClient.call(restSrv);
  restFlag = false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "JoyRemote");
  ros::NodeHandle nh;

  JoyRemote joyTest(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    joyTest.sendCmd();
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}

