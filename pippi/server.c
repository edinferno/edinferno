#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "network.h"

#define PORT 7890  // The port users will be connecting to

int main(void) {
    int sockfd;  // Receive data on sock_fd
    struct sockaddr_in host_addr;  // My address information
    socklen_t sin_size;
    int recv_length = 1, yes = 1;
    char buffer[1024];

    printf("server: Initializing socket...\n");

    // UDP connection (SOCK_DGRAM) over IPv4 (PF_INET); 0 means protocol as the
    // specification allows multiple protocols for a protocol family; for IPv4,
    // however, there is a single protocol, so 0.
    if ((sockfd = socket(PF_INET, SOCK_DGRAM, 0)) == -1)
        fatal("server: fatal error in socket!");

    printf("server: Setting SO_REUSEADDR...\n");

    // Allow socket to be reused for binding (SO_REUSEADDR). SOL_SOCKET is the
    // level of the option being set; &yes is a pointer to the value for the
    // option (1 meaning true); size(int) is the size of the data stored at the
    // given pointer.
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
        fatal("server: fatal error while setting socket option SO_REUSEADDR!");

    host_addr.sin_family = AF_INET;          // Host byte order
    host_addr.sin_port = htons(PORT);        // Short, network byte order
    host_addr.sin_addr.s_addr = 0;           // Automatically fill with my IP.
    memset(&(host_addr.sin_zero), '\0', 8);  // Zero the rest of the struct.

    printf("server: Binding to socket...\n");

    // Bind socket to address
    if (bind(sockfd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr)) == -1)
        fatal("server: fatal error while binding to socket!");

    printf("server: Receiving messages...\n");

    while (1) {  // Receive loop
        recv_length = recv(sockfd, &buffer, 1024, 0);
        if (recv_length > 0) {
            printf("server: Message received: [%s].\n", (char *)&buffer);
        }
        if (recv_length == -1)
            fatal("server: fatal error while receiving messages!");
    }
    return 0;
}
