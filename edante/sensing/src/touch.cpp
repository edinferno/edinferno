/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's touch sensors
*/

#include "touch.h"

Touch::Touch(ros::NodeHandle* nh, AL::ALMemoryProxy* memProxy) {
  nh_ = nh;
  memProxy_ = memProxy;
  INFO("Setting up Touch publishers" << std::endl);
  bumpers_pub_ = nh_->advertise<sensing::bumpers>("bumpers", 10);
  chest_pub_ = nh_->advertise<std_msgs::Bool>("chest", 10);
  head_pub_ = nh_->advertise<sensing::head>("head", 10);
  right_hand_pub_ = nh_->advertise<sensing::hand>("rightHand", 10);
  left_hand_pub_ = nh_->advertise<sensing::hand>("leftHand", 10);
}

Touch::~Touch() {
  ros::shutdown();
}

void Touch::spinTopics() {
  this->checkBumpers();
  this->checkChest();
  this->checkHead();
  this->checkHands();
}

void Touch::checkBumpers() {
  sensing::bumpers msg;
  if (float(memProxy_->getData("RightBumperPressed")) > 0.5) {
    msg.right = true;
  } else {
    msg.right = false;
  }
  if (float(memProxy_->getData("LeftBumperPressed")) > 0.5) {
    msg.left = true;
  } else {
    msg.left = false;
  }
  bumpers_pub_.publish(msg);
}

void Touch::checkChest() {
  std_msgs::Bool msg;
  if (float(memProxy_->getData("ChestButtonPressed")) > 0.5) {
    msg.data = true;
  } else {
    msg.data = false;
  }
  chest_pub_.publish(msg);
}

void Touch::checkHead() {
  sensing::head msg;
  if (float(memProxy_->getData("FrontTactilTouched")) > 0.5) {
    msg.front = true;
  } else {
    msg.front = false;
  }
  if (float(memProxy_->getData("MiddleTactilTouched")) > 0.5) {
    msg.middle = true;
  } else {
    msg.middle = false;
  }
  if (float(memProxy_->getData("RearTactilTouched")) > 0.5) {
    msg.rear = true;
  } else {
    msg.rear = false;
  }
  head_pub_.publish(msg);
}

void Touch::checkHands() {
  this->checkRightHand();
  this->checkLeftHand();
}

void Touch::checkRightHand() {
  sensing::hand msg;
  if (float(memProxy_->getData("HandRightBackTouched")) > 0.5) {
    msg.back = true;
  } else {
    msg.back = false;
  }
  if (float(memProxy_->getData("HandRightLeftTouched")) > 0.5) {
    msg.left = true;
  } else {
    msg.left = false;
  }
  if (float(memProxy_->getData("HandRightRightTouched")) > 0.5) {
    msg.right = true;
  } else {
    msg.right = false;
  }
  right_hand_pub_.publish(msg);
}

void Touch::checkLeftHand() {
  sensing::hand msg;
  if (float(memProxy_->getData("HandLeftBackTouched")) > 0.5) {
    msg.back = true;
  } else {
    msg.back = false;
  }
  if (float(memProxy_->getData("HandLeftLeftTouched")) > 0.5) {
    msg.left = true;
  } else {
    msg.left = false;
  }
  if (float(memProxy_->getData("HandLeftRightTouched")) > 0.5) {
    msg.right = true;
  } else {
    msg.right = false;
  }
  left_hand_pub_.publish(msg);
}
