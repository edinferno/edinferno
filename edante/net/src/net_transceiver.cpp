/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-19
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The NetTransceiver class can send and received the SPL Robocup
*             messages. It can recieve game controller data and answer
*             appropriately; receive and send SPL standard messages;
*             broadcast coach messages.
*/
#include "net/net_transceiver.hpp"

#include <string.h>
#include <netinet/in.h>

NetTransceiver::NetTransceiver() {
  // Game controller socket
  game_data_sd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  int yes = 1;
  setsockopt(game_data_sd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

  // Set timeout time of 1ms
  struct timeval read_timeout;
  read_timeout.tv_sec = 0;
  read_timeout.tv_usec = 1000 * 1;  // 1 ms
  setsockopt(game_data_sd_, SOL_SOCKET, SO_RCVTIMEO,
             &read_timeout,
             sizeof(read_timeout));

  // Bind the socket to the corresponding port regardless of the sender IP.
  sockaddr_in game_data_server;
  memset(&game_data_server, 0, sizeof(game_data_server));
  game_data_server.sin_family = AF_INET;
  game_data_server.sin_port = htons(kGameDataPort);
  game_data_server.sin_addr.s_addr = INADDR_ANY;
  bind(game_data_sd_,
       reinterpret_cast<sockaddr*>(&game_data_server),
       sizeof(sockaddr));
}

bool NetTransceiver::ReceiveGameData(
  const RoboCupGameControlReturnData& return_data,
  RoboCupGameControlData& game_data) {
  sockaddr_in game_controller;

  socklen_t sockaddr_len = sizeof(sockaddr);
  ssize_t len;
  bool data_available = false;

  // Read all received messages up to the last one, otherwise
  // only the first message in the queue (the oldest) will be read.
  // If the function is called often enough there should always be
  // only one message in the queue. The game controller broadcasts
  // 5 times per second.
  do {
    len = recvfrom(game_data_sd_,
                   &game_data,
                   sizeof(game_data),
                   0,
                   reinterpret_cast<sockaddr*>(&game_controller),
                   &sockaddr_len);
    if (len > 0) data_available = true;
  } while (len > 0);

  // If new game controller data was received
  if (data_available < 0) {
    return false;
  }

  // Respond to the game controller
  game_controller.sin_port = htons(kReturnGameDataPort);
  sendto(game_data_sd_,
         &return_data,
         sizeof(return_data),
         0,
         reinterpret_cast<sockaddr*>(&game_controller),
         sizeof(sockaddr));

  return true;
}
