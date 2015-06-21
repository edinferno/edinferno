/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-21
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The CommsNode publishes the received information over UDP in ROS
*/
#include "comms/comms_node.hpp"

#include <std_msgs/UInt8.h>

CommsNode::CommsNode() :
  nh_("comms"),
  game_state_pub_(nh_.advertise<std_msgs::UInt8>("game_state", 1)),
  penalised_pub_(nh_.advertise<std_msgs::UInt8>("penalized", 1)),
  team_color_pub_(nh_.advertise<std_msgs::UInt8>("team_color", 1)) {
  // TODO(svepe): Read ROS params
  team_number_ = 9;
  player_number_ = 1;
  game_return_data_.team = 9;
  game_return_data_.player = 1;
  game_return_data_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
}

void CommsNode::Spin() {
  ros::Rate r(5);
  while (ros::ok()) {
    if (net_.ReceiveGameData(game_return_data_, game_data_)) {
      Publish();
    }
    r.sleep();
  }
}

void CommsNode::Publish() {
  PublishGameState();
  PublishPenalised();
  PublishTeamColor();
}

void CommsNode::PublishGameState() {
  std_msgs::UInt8 msg;
  msg.data = game_data_.state;
  game_state_pub_.publish(msg);
}

void CommsNode::PublishPenalised() {
  std_msgs::UInt8 msg;
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  msg.data = game_data_.teams[t].players[player_number_ - 1].penalty;
  penalised_pub_.publish(msg);
}

void CommsNode::PublishTeamColor() {
  std_msgs::UInt8 msg;
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  msg.data = game_data_.teams[t].teamColour;
  team_color_pub_.publish(msg);
}
