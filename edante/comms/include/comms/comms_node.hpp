/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-21
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The CommsNode publishes the received information over UDP in ROS
*/
/// Implementation of the Communication module from the Architecture Diagram.
///
/// Coordinate systems used
///  * Absolute coordinates (x, y) or (x, y, θ):
///     - x is the coordinate on Ox, which points from the center of the pitch
///       to the center of the opponent’s goal
///     - y is the coordinate on Oy, which is perpendicular to Ox and pointing
///       to the left when looking in the direction of Ox
///     - θ is a standard orientation in the [Ox, Oy] coordinate system
///       (increasing counter clockwise)
///     - linear measurements (x, y) are in mm; angular measurements θ are in
///       radians
///  * Relative coordinates (x, y) or (vx, vy):
///     - x is the coordinate on Ox, which points from the center of the robot
///       to the direction at which its body is facing
///     - y is the coordinate on Oy, which is perpendicular to Ox and pointing
///       to the left when looking in the direction of Ox (increasing counter
///       clockwise)
///     - spatial measurements (x, y) are in mm; speed measurements (vx, vy) are
///       in mm/ms
///  * Coordinated team time (CTT) - the temporal coordinate system for
///    timestamps:
///     - the time elapsed since the last synchronization of the clocks of the
///       team.
#ifndef COMMS_NODE_H
#define COMMS_NODE_H

#include <vector>

#include <ros/ros.h>

#include <std_msgs/UInt8.h>

#include <spl_msgs/RoboCupGameControlData.h>
#include <spl_msgs/SPLStandardMessage.h>

#include <net/net_transceiver.hpp>

class CommsNode {
 public:
  CommsNode();
  void Spin();

 private:
  static const uint8_t kStateUnknown = 255;
  static const uint8_t kPenaltyUnknown = 255;
  static const uint8_t kColorUnknown = 255;

  int team_number_;
  int player_number_;

  ros::NodeHandle nh_;

  ros::Publisher game_state_pub_;
  std_msgs::UInt8 game_state_msg_;

  ros::Publisher penalised_pub_;
  std_msgs::UInt8 penalised_msg_;

  ros::Publisher team_color_pub_;
  std_msgs::UInt8 team_color_msg_;

  NetTransceiver net_;

  RoboCupGameControlData game_data_;
  RoboCupGameControlReturnData game_return_data_;

  SPLStandardMessage send_msg_;
  std::vector<SPLStandardMessage> recv_msgs_;

  void Publish();
  void PublishGameState();
  void PublishPenalised();
  void PublishTeamColor();
};

#endif
