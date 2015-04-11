#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "NetReceiver.h"

NetReceiver::NetReceiver() {
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

SPLStandardMessage NetReceiver::receiveStandardMessage() {
    bind(recv_socket_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr));

    SPLStandardMessage stdMsg;
    int recv_length = recv(recv_socket_fd, &stdMsg, sizeof(stdMsg), 0);

    if (recv_length > 0) {
        printf("server: Message received; length [%d].\n", recv_length);
    }
    return stdMsg;
}