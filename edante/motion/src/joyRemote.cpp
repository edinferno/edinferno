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
  moveClient = nh_->serviceClient<motion::moveToward>("/motion/moveToward", true);
  INFO("Joystick remote node initialised" << std::endl);
}

joyRemote::~joyRemote() {
  ros::shutdown();
}

void joyRemote::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  float x = msg->axes[1];
  float y = msg->axes[0];
  float theta = msg->axes[3];
  moveSrv.request.normVelocity.x = x;
  moveSrv.request.normVelocity.y = y;
  moveSrv.request.normVelocity.theta = theta;
}

void joyRemote::sendCmd() {
  moveClient.call(moveSrv);
}

int main(int argc, char *argv[]) {
  using namespace std;
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

