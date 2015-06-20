/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The NetTransceiver class is a bridge between ROS and UDP messages.
*/
#ifndef NET_TRANSCEIVER_HPP
#define NET_TRANSCEIVER_HPP

#include <netinet/in.h>

#include "net/RoboCupGameControlData.h"
#include "net/SPLStandardMessage.h"
#include "net/SPLCoachMessage.h"

class NetTransceiver {
 public:
  NetTransceiver();
  // Game Controller communication
  bool ReceiveGameData(const RoboCupGameControlReturnData& return_data,
                       RoboCupGameControlData& game_data);
  // SPL Standard message support
  bool BroadcastSPLStandardMessage(const SPLStandardMessage& msg);
  bool ReceiveSPLStandardMessage(SPLStandardMessage& msg);

  // SPL Coach message support
  bool BroadcastSPLCoachMessage(const SPLCoachMessage& msg);

 private:
  static const int kGameDataPort = GAMECONTROLLER_PORT;
  static const int kReturnGameDataPort = GAMECONTROLLER_PORT;
  static const int kTeamBroadcastPort = 10009;
  static const int kCoachBroadcastPort = SPL_COACH_MESSAGE_PORT;

  int game_data_sd_;

  int boradcast_spl_sd_;
  sockaddr_in broadcast_spl_server_;

  int boradcast_coach_sd_;
  sockaddr_in broadcast_coach_server_;
};
#endif
