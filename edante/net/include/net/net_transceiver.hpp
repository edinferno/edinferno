/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The NetTransceiver class is a bridge between ROS and UDP messages.
*/
#ifndef NET_TRANSCEIVER_HPP
#define NET_TRANSCEIVER_HPP

#include "net/RoboCupGameControlData.h"
class NetTransceiver {
 public:
  NetTransceiver();
  bool ReceiveGameData(const RoboCupGameControlReturnData& return_data,
                       RoboCupGameControlData& game_data);
 private:
  static const int kGameDataPort = 3838;
  static const int kReturnGameDataPort = 3838;

  int game_data_sd_;
};
#endif
