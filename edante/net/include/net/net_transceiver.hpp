/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The NetTransceiver class is a bridge between ROS and UDP messages.
*/
#ifndef NET_TRANSCEIVER_HPP
#define NET_TRANSCEIVER_HPP

#include <netinet/in.h>

#include <vector>

#include "spl_msgs/RoboCupGameControlData.h"
#include "spl_msgs/SPLStandardMessage.h"
#include "spl_msgs/SPLCoachMessage.h"

class NetTransceiver {
 public:
  explicit NetTransceiver(int team_number);
  // Game Controller communication
  bool ReceiveGameData(const RoboCupGameControlReturnData& return_data,
                       RoboCupGameControlData& game_data);
  // SPL Standard message support
  bool BroadcastSPLStandardMessage(const SPLStandardMessage& msg);
  bool ReceiveSPLStandardMessage(std::vector<SPLStandardMessage>& msgs);

  // SPL Coach message support
  bool BroadcastSPLCoachMessage(const SPLCoachMessage& msg);

 private:
  int game_data_sd_;

  int boradcast_spl_sd_;
  sockaddr_in broadcast_spl_server_;

  int boradcast_coach_sd_;
  sockaddr_in broadcast_coach_server_;
};
#endif
