#include "SPLStandardMessage.h"
#include "Networking.h"

int main() {
    SPLStandardMessage splMsg;
    Networking net;
    net.sendStandardMessage(splMsg);
}
