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
  kickoff_attack_pub_(nh_.advertise<std_msgs::Bool>("kickoff_attack", 1, true)),
  man_penalised_sub_(nh_.subscribe(
                       "/world/manually_penalized", 1,
                       &CommsNode::ManuallyPenalisedCallback, this)),
  has_fallen_sub_(nh_.subscribe(
                    "/world/has_fallen", 1,
                    &CommsNode::HasFallenCallback, this)),
  intention_sub_(nh_.subscribe("/world/intention", 1,
                               &CommsNode::IntentionCallback, this)),
  robot_pose_sub_(nh_.subscribe("/world/robot_pose", 1,
                                &CommsNode::RobotPoseCallback, this)),
  walking_to_sub_(nh_.subscribe("/world/walking_to", 1,
                                &CommsNode::WalkingToCallback, this)),
  first_game_data_(true) {

  nh_.getParam("/player_number", player_number_);
  nh_.getParam("/team_number", team_number_);
  net_ = new NetTransceiver(team_number_);

  InitSPLMessage();

  game_return_data_.team = team_number_;
  game_return_data_.player = player_number_;
  game_return_data_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
}

CommsNode::~CommsNode() {
  delete net_;
}

void CommsNode::Spin() {
  ros::Rate r(4);
  while (ros::ok()) {
    if (net_->ReceiveGameData(game_return_data_, game_data_)) {
      Publish();
    }
    net_->BroadcastSPLStandardMessage(send_msg_);
    ros::spinOnce();
    r.sleep();
  }
}

void CommsNode::InitSPLMessage() {
  send_msg_.playerNum = player_number_;
  send_msg_.teamNum = team_number_;

  send_msg_.fallen = false;

  send_msg_.pose[0] = 0;
  send_msg_.pose[1] = 0;
  send_msg_.pose[2] = 0;

  send_msg_.walkingTo[0] = 0;
  send_msg_.walkingTo[1] = 0;

  send_msg_.shootingTo[0] = 4.5;
  send_msg_.shootingTo[0] = 0;

  send_msg_.ballAge = -1;

  send_msg_.ball[0] = 0;
  send_msg_.ball[1] = 0;

  send_msg_.ballVel[0] = 0;
  send_msg_.ballVel[1] = 0;

  for (int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
    send_msg_.suggestion[i] = 0;

  send_msg_.intention = 0;

  send_msg_.averageWalkSpeed = 100;

  send_msg_.maxKickDistance = 4000;

  send_msg_.currentPositionConfidence = 0;
  send_msg_.currentSideConfidence = 0;

  send_msg_.numOfDataBytes = 0;
}

void CommsNode::Publish() {
  PublishGameState();
  PublishPenalised();
  PublishTeamColor();
  PublishKickoffAttack();
  first_game_data_ = false;
}

void CommsNode::PublishGameState() {
  if (game_state_msg_.data != game_data_.state || first_game_data_) {
    game_state_msg_.data = game_data_.state;
    game_state_pub_.publish(game_state_msg_);
  }
}

void CommsNode::PublishPenalised() {
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  int penalty = game_data_.teams[t].players[player_number_ - 1].penalty;
  if (penalised_msg_.data != penalty || first_game_data_) {
    penalised_msg_.data = penalty;
    penalised_pub_.publish(penalised_msg_);
  }
}

void CommsNode::PublishTeamColor() {
  int t = (game_data_.teams[0].teamNumber == team_number_) ? 0 : 1;
  int color = game_data_.teams[t].teamColour;
  if (team_color_msg_.data != color || first_game_data_)  {
    team_color_msg_.data = color;
    team_color_pub_.publish(team_color_msg_);
  }
}

void CommsNode::PublishKickoffAttack() {
  bool attack = (game_data_.kickOffTeam == team_number_);
  if (kickoff_attack_msg_.data != attack || first_game_data_) {
    kickoff_attack_msg_.data = attack;
    kickoff_attack_pub_.publish(kickoff_attack_msg_);
  }
}

void CommsNode::ManuallyPenalisedCallback(const std_msgs::UInt8& msg) {
  game_return_data_.message = msg.data;
}

void CommsNode::HasFallenCallback(const std_msgs::Bool& msg) {
  send_msg_.fallen = msg.data;
}

void CommsNode::IntentionCallback(const std_msgs::Int8& msg) {
  send_msg_.intention = msg.data;
}

void CommsNode::RobotPoseCallback(const geometry_msgs::Pose2D& msg) {
  send_msg_.pose[0] = static_cast<float>(msg.x);
  send_msg_.pose[1] = static_cast<float>(msg.y);
  send_msg_.pose[2] = static_cast<float>(msg.theta);
}

void CommsNode::WalkingToCallback(const geometry_msgs::Pose2D& msg) {
  send_msg_.walkingTo[0] = static_cast<float>(msg.x);
  send_msg_.walkingTo[1] = static_cast<float>(msg.y);
}

