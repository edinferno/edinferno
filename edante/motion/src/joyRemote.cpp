/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Joystick remote control node
*/

#include "joyRemote.h"

joyRemote::joyRemote(ros::NodeHandle* nh) {
  nh_ = nh;
  joy_sub_ = nh_->subscribe("joy", 1000, &joyRemote::joyCallback, this);
  moveTowardClient = nh_->serviceClient<motion::moveToward>(
                       "/motion/moveToward", true);
  wakeUpClient = nh_->serviceClient<std_srvs::Empty>(
                   "/motion/wakeUp", true);
  moveInitClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/moveInit", true);
  standClient = nh_->serviceClient<motion::setPosture>(
                  "/motion/goToPosture", true);
  restClient = nh_->serviceClient<std_srvs::Empty>(
                 "/motion/rest", true);
  stopMoveClient = nh_->serviceClient<std_srvs::Empty>(
                     "/motion/stopMove", true);
  INFO("Joystick remote node initialised" << std::endl);
  wakeUpFlag = false;
  moveInitFlag = false;
  standFlag = false;
  restFlag = false;
  standSrv.request.speed = 1.0f;
}

joyRemote::~joyRemote() {
  ros::shutdown();
}

void joyRemote::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
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
    moveSrv.request.normVelocity.x = x;
    moveSrv.request.normVelocity.y = y;
    moveSrv.request.normVelocity.theta = theta;
  }
}

void joyRemote::sendCmd() {
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

void joyRemote::wakeUp() {
  wakeUpClient.call(wakeUpSrv);
  wakeUpFlag = false;
}

void joyRemote::moveInit() {
  moveInitClient.call(moveInitSrv);
  moveInitFlag = false;
}

void joyRemote::stand() {
  wakeUpClient.call(wakeUpSrv);
  standSrv.request.postureName = "Stand";
  standClient.call(standSrv);
  moveInitClient.call(moveInitSrv);
  standFlag = false;
}

void joyRemote::rest() {
  stopMoveClient.call(stopSrv);
  sleep(0.25);
  moveInitClient.call(moveInitSrv);
  sleep(0.25);
  restClient.call(restSrv);
  restFlag = false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "joyRemote");
  ros::NodeHandle nh;

  joyRemote joyTest(&nh);

  ros::Rate r(5);

  while (ros::ok()) {
    joyTest.sendCmd();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

