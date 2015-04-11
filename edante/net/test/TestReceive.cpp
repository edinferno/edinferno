#include "SPLStandardMessage.h"
#include "NetReceiver.h"

#include <iostream>

using std::cout;

int main() {
    SPLStandardMessage splMsg;
    NetReceiver net;
    splMsg = net.receiveStandardMessage();
    cout << splMsg;
}
