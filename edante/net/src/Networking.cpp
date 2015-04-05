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

        // Init address and port
        inet_aton(TEAMMATE_ADDRESS[i].c_str(), &target_in_addr);

        // Fill target_addr struct
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(TEAMMATE_PORT[i]);
        target_addr.sin_addr = target_in_addr;
        memset(&(target_addr.sin_zero), '\0', 8);

        teammate_addresses.push_back(target_addr);
        // Potential improvements: use an array to store the addresses
    }

    // Init receiver
    int yes = 1;
    recv_socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    setsockopt(recv_socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    // Init host address structure
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(MY_PORT);
    my_addr.sin_addr.s_addr = 0;           // Automatically fill with my IP.
    memset(&(my_addr.sin_zero), '\0', 8);
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

SPLStandardMessage Networking::receiveStandardMessage() {
    bind(recv_socket_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr));

    SPLStandardMessage stdMsg;
    int recv_length = recv(recv_socket_fd, &stdMsg, sizeof(stdMsg), 0);

    if (recv_length > 0) {
        printf("server: Message received; length [%d].\n", recv_length);
    }
    return stdMsg;
}
