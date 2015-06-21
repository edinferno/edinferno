/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-21
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The CommsNode publishes the received information over UDP in ROS
*/
#include "comms/comms_node.hpp"

CommsNode::CommsNode() :
  nh_("comms"),
  game_state_pub_(nh_.advertise<std_msgs::UInt8>("game_state", 1, true)),
  penalised_pub_(nh_.advertise<std_msgs::UInt8>("penalized", 1, true)),
  team_color_pub_(nh_.advertise<std_msgs::UInt8>("team_color", 1, true)),
  man_penalised_sub_(nh_.subscribe(
                       "/world/manually_penalized",
                       1,
                       &CommsNode::ManuallyPenalisedCallback, this)) {
  // TODO(svepe): Read ROS params
  team_number_ = 9;
  player_number_ = 1;
  game_return_data_.team = 9;
  game_return_data_.player = 1;
  game_return_data_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

  game_state_msg_.data = kStateUnknown;
  penalised_msg_.data = kPenaltyUnknown;
  team_color_msg_.data = kColorUnknown;
}

void CommsNode::Spin() {
  ros::Rate r(5);
  while (ros::ok()) {
    if (net_.ReceiveGameData(game_return_data_, game_data_)) {
      Publish();
    }
    ros::spinOnce();
    r.sleep();
  }
}

void CommsNode::Publish() {
  PublishGameState();
  PublishPenalised();
  PublishTeamColor();
}

void CommsNode::PublishGameState() {
  if (game_state_msg_.data != game_data_.state) {
    game_state_msg_.data = game_data_.state;
    game_state_pub_.publish(game_state_msg_);
  }
}

void CommsNode::PublishPenalised() {
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  int penalty = game_data_.teams[t].players[player_number_ - 1].penalty;
  if (penalised_msg_.data != penalty) {
    penalised_msg_.data = penalty;
    penalised_pub_.publish(penalised_msg_);
  }
}

void CommsNode::PublishTeamColor() {
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  if (team_color_msg_.data != game_data_.teams[t].teamColour)  {
    team_color_msg_.data = game_data_.teams[t].teamColour;
    team_color_pub_.publish(team_color_msg_);
  }
}

void CommsNode::ManuallyPenalisedCallback(const std_msgs::UInt8& msg) {
  game_return_data_.message = msg.data;
}
