/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Tests the net transceiver.
*/

#include <unistd.h>

#include <iostream>

#include "net/net_transceiver.hpp"
#include "net/RoboCupGameControlData.h"

int main(int argc, const char** argv) {
  RoboCupGameControlReturnData return_data;
  RoboCupGameControlData game_data;

  NetTransceiver net;

  return_data.team = 9;
  return_data.player = 1;

  while (1) {
    bool r = net.ReceiveGameData(return_data, game_data);
    if (r)
      std::cout << "Game Controller state: " << (int)game_data.state << std::endl;
    else
      std::cout << "No data" << std::endl;
    sleep(1);
  }




  return 0;
}
