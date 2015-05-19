/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Joystick remote control node
*/

#include "joy_remote.h"

JoyRemote::JoyRemote(ros::NodeHandle* nh) {
  nh_ = nh;
  joy_sub_ = nh_->subscribe("joy", 1000, &JoyRemote::joyCallback, this);
  moveTowardClient = nh_->serviceClient<motion::MoveToward>(
                       "/motion/move_toward", true);
  wakeUpClient = nh_->serviceClient<std_srvs::Empty>(
                   "/motion/wake_up", true);
  moveInitClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/move_init", true);
  standClient = nh_->serviceClient<motion::SetPosture>(
                  "/motion/goto_posture", true);
  restClient = nh_->serviceClient<std_srvs::Empty>(
                 "/motion/rest", true);
  stopMoveClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/stop_move", true);
  INFO("Joystick remote node initialised" << std::endl);
  wakeUpFlag = false;
  moveInitFlag = false;
  standFlag = false;
  restFlag = false;
  standSrv.request.speed = 1.0f;
}

JoyRemote::~JoyRemote() {
  ros::shutdown();
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
  } else {moveTowardClient.call(moveSrv);}
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
  moveInitClient.call(moveInitSrv);
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

  ros::Rate r(5);

  while (ros::ok()) {
    joyTest.sendCmd();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

