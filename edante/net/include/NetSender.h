/// Implementation of the Networking module from the Architecure Diagram.
/// The SPL Standard Message is created by the Communication Module
/// and then sent to the Networking module via ROS.

#ifndef EDANTE_NET_H
#define EDANTE_NET_H

#include <netinet/in.h>
#include <vector>
using std::vector;
#include <string>
using std::string;

#include "SPLStandardMessage.h"

const int            NUM_OF_TEAMMATES = 4;
const vector<string> TEAMMATE_ADDRESS = {
                         "127.0.0.1",
                         "127.0.0.1",
                         "127.0.0.1",
                         "127.0.0.1"
                     };
const vector<int>    TEAMMATE_PORT    = {
                         7890,
                         7891,
                         7892,
                         7893
                     };

class NetSender {
private:
    int send_socket_fd;
    vector<struct sockaddr_in> teammate_addresses;

public:
    NetSender();

    /// Sends the specified standard message to all teammates.
    void sendStandardMessage(SPLStandardMessage splMsg);
};

#endif