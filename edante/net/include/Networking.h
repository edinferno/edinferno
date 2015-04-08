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

const int            MY_PORT = 7890;

class Networking {
private:
    int send_socket_fd;
    int recv_socket_fd;

    vector<struct sockaddr_in> teammate_addresses;
    struct sockaddr_in my_addr;
public:
    Networking();

    /// Sends the specified standard message to all teammates.
    void sendStandardMessage(SPLStandardMessage splMsg);

    /// Receives a standard message from a teammate and returns it.
    SPLStandardMessage receiveStandardMessage();
    // Possible optimization: Try to use const & and see if performance is
    // better.
};

#endif
