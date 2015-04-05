#include "SPLStandardMessage.h"
#include "Networking.h"

#include <iostream>

using std::cout;

int main() {
    SPLStandardMessage splMsg;
    Networking net;
    splMsg = net.receiveStandardMessage();
    cout << splMsg;
}
