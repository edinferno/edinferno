/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Tests the net transceiver.
*/

#include <unistd.h>

#include <iostream>

#include "net/net_transceiver.hpp"

int main(int argc, const char** argv) {
  RoboCupGameControlReturnData return_data;
  RoboCupGameControlData game_data;

  NetTransceiver net;
  SPLStandardMessage spl_msg;
  SPLStandardMessage spl_msg_rec;
  SPLCoachMessage cch_msg;

  return_data.team = 9;
  return_data.player = 1;

  spl_msg.teamNum = 9;
  spl_msg.playerNum = 1;

  cch_msg.team = 9;

  while (1) {
    std::cout << "Receive game data status: "
              << net.ReceiveGameData(return_data, game_data) << std::endl;
    std::cout << "Receive SPL: "
              << net.ReceiveSPLStandardMessage(spl_msg_rec) << std::endl;
    std::cout << "Broadcast SPL status: "
              << net.BroadcastSPLStandardMessage(spl_msg) << std::endl;
    std::cout << "Broadcast CCH status: "
              << net.BroadcastSPLCoachMessage(cch_msg) << std::endl;
    sleep(1);
  }
  return 0;
}
