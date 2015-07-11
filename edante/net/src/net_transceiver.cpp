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

#include <arpa/inet.h>

NetTransceiver::NetTransceiver(int team_number) {
  // Socket option
  int yes = 1;
  // Set read timeout to 1ms
  struct timeval read_timeout;
  read_timeout.tv_sec = 0;
  read_timeout.tv_usec = 1000 * 1;  // 1 ms

  // Game controller socket
  game_data_sd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(game_data_sd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  setsockopt(game_data_sd_, SOL_SOCKET, SO_RCVTIMEO,
             &read_timeout,
             sizeof(read_timeout));
  sockaddr_in game_data_server;
  memset(&game_data_server, 0, sizeof(game_data_server));
  game_data_server.sin_family = AF_INET;
  game_data_server.sin_port = htons(GAMECONTROLLER_PORT);
  game_data_server.sin_addr.s_addr = INADDR_ANY;
  bind(game_data_sd_,
       reinterpret_cast<sockaddr*>(&game_data_server),
       sizeof(sockaddr));

  // SPL broadcast socket
  boradcast_spl_sd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(boradcast_spl_sd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  setsockopt(boradcast_spl_sd_, SOL_SOCKET, SO_RCVTIMEO,
             &read_timeout,
             sizeof(read_timeout));
  memset(&broadcast_spl_server_, 0, sizeof(broadcast_spl_server_));
  broadcast_spl_server_.sin_family = AF_INET;
  broadcast_spl_server_.sin_port = htons(10000 + team_number);
  broadcast_spl_server_.sin_addr.s_addr = inet_addr("255.255.255.255");
  bind(boradcast_spl_sd_,
       reinterpret_cast<sockaddr*>(&broadcast_spl_server_),
       sizeof(sockaddr));

  // Coach broadcast socket
  boradcast_coach_sd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(boradcast_coach_sd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  setsockopt(boradcast_coach_sd_, SOL_SOCKET, SO_RCVTIMEO,
             &read_timeout,
             sizeof(read_timeout));
  memset(&broadcast_coach_server_, 0, sizeof(broadcast_coach_server_));
  broadcast_coach_server_.sin_family = AF_INET;
  broadcast_coach_server_.sin_port = htons(SPL_COACH_MESSAGE_PORT);
  broadcast_coach_server_.sin_addr.s_addr = inet_addr("255.255.255.255");
  bind(boradcast_coach_sd_,
       reinterpret_cast<sockaddr*>(&broadcast_coach_server_),
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
  if (!data_available) {
    return false;
  }

  // Respond to the game controller
  game_controller.sin_port = htons(GAMECONTROLLER_PORT);
  sendto(game_data_sd_,
         &return_data,
         sizeof(return_data),
         0,
         reinterpret_cast<sockaddr*>(&game_controller),
         sizeof(sockaddr));

  return true;
}

bool NetTransceiver::BroadcastSPLStandardMessage(
  const SPLStandardMessage& msg) {
  ssize_t len = sendto(boradcast_spl_sd_,
                       &msg,
                       sizeof(msg),
                       0,
                       reinterpret_cast<sockaddr*>(&broadcast_spl_server_),
                       sizeof(sockaddr));
  return (len > 0);
}

bool NetTransceiver::ReceiveSPLStandardMessage(
  std::vector<SPLStandardMessage>& msgs) {
  sockaddr_in sender;
  socklen_t sockaddr_len = sizeof(sockaddr);
  ssize_t len;
  SPLStandardMessage msg;
  msgs.clear();

  do {
    len = recvfrom(boradcast_spl_sd_,
                   &msg,
                   sizeof(msg),
                   0,
                   reinterpret_cast<sockaddr*>(&sender),
                   &sockaddr_len);
    if (len > 0) {
      msgs.push_back(msg);
    }
  } while (len > 0);

  return (msgs.size() != 0);
}

bool NetTransceiver::BroadcastSPLCoachMessage(const SPLCoachMessage& msg) {
  ssize_t len = sendto(boradcast_coach_sd_,
                       &msg,
                       sizeof(msg),
                       0,
                       reinterpret_cast<sockaddr*>(&broadcast_coach_server_),
                       sizeof(sockaddr));
  return (len > 0);
}
