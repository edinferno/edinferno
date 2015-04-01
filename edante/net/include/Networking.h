/// Implementation of the Communication module from the Architecure Diagram.
/// The SPL Standard Message is created by the Commuication Module
/// and then sent to the Communication module via ROS.

#ifndef EDANTE_NET_H
#define EDANTE_NET_H


#include "SPLStandardMessage.h"
class Networking {
public:

    /// Sends the specified standard message to all teammates.
    void sendStandardMessage(SPLStandardMessage splMsg);

    /// Receives a standard message from a teammate and returns it.
    void receiveStandardMessage(SPLStandardMessage splMsg);
};

#endif