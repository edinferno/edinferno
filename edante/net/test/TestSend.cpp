#include "SPLStandardMessage.h"
#include "NetSender.h"

int main() {
    SPLStandardMessage splMsg;
    NetSender net;
    net.sendStandardMessage(splMsg);
}
