/// Implementation of the Networking module from the Architecure Diagram.
/// The SPL Standard Message is created by the Communication Module
/// and then sent to the Networking module via ROS.

#ifndef EDANTE_NET_H
#define EDANTE_NET_H

#include <netinet/in.h>
#include <string>
#include <vector>




using std::string;
using std::vector;

#include "SPLStandardMessage.h"

const int            MY_PORT = 7890;

class NetReceiver {
 private:
  int recv_socket_fd;
  struct sockaddr_in my_addr;
 public:
  NetReceiver();

  /// Receives a standard message from a teammate and returns it.
  SPLStandardMessage receiveStandardMessage();
  // Possible optimization: Try to use const & and see if performance is
  // better.
};

#endif
