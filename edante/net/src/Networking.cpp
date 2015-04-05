#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "Networking.h"

Networking::Networking() {
    send_socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    // Create team mate address structures
    for (int i = 0; i < NUM_OF_TEAMMATES; ++i) {
        struct sockaddr_in target_addr;
        struct in_addr     target_in_addr;
        uint16_t           target_in_port;

        // Init address and port
        inet_aton(TEAMMATE_ADDRESS[i].c_str(), &target_in_addr);
        target_in_port = TEAMMATE_PORT[i];

        // Fill target_addr struct
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(target_in_port);
        target_addr.sin_addr = target_in_addr;
        memset(&(target_addr.sin_zero), '\0', 8);

        teammate_addresses.push_back(target_addr);
        // Potential improvements: use an array to store the addresses
    }
}

void Networking::sendStandardMessage(SPLStandardMessage splMsg) {
    for (int i = 0; i < NUM_OF_TEAMMATES; ++i) {
        int bytes = sendto(
            send_socket_fd,
            &splMsg, sizeof(splMsg),
            0,
            (struct sockaddr *) &teammate_addresses[i], sizeof(struct sockaddr)
        );
    }
}