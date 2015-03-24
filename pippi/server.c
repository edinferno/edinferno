#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "network.h"

#define PORT 7890  // The port users will be connecting to

int main(void) {
    int sockfd, new_sockfd;  // Listen on sock_fd, new connection on new_fd
    struct sockaddr_in host_addr, client_addr;  // My address information
    socklen_t sin_size;
    int recv_length = 1, yes = 1;
    char buffer[1024];

    // TCP connection (SOCK_STREAM) over IPv4 (PF_INET); 0 means protocol as the
    // specification allows multiple protocols for a protocol family; for IPv4,
    // however, there is a single protocol, so 0.
    if (-1 == (sockfd = socket(PF_INET, SOCK_STREAM, 0)))
        fatal("server: fatal error in socket!");

    // Allow socket to be reused for binding (SO_REUSEADDR). SOL_SOCKET is the
    // level of the option being set; &yes is a pointer to the value for the
    // option (1 meaning true); size(int) is the size of the data stored at the
    // given pointer.
    if (-1 == setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)))
        fatal("server: fatal error while setting socket option SO_REUSEADDR!");

    host_addr.sin_family = AF_INET;          // Host byte order
    host_addr.sin_port = htons(PORT);        // Short, network byte order
    host_addr.sin_addr.s_addr = 0;           // Automatically fill with my IP.
    memset(&(host_addr.sin_zero), '\0', 8);  // Zero the rest of the struct.

    // Bind socket to address
    if (-1 == bind(sockfd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr)))
        fatal("server: fatal error while binding to socket!");

    // Listen for incoming connections; 5 is the size of the backlog queue
    if (-1 == listen(sockfd, 5))
        fatal("server: fatal error while listening on socket!");

    while (1) {  // Accept loop
        sin_size = sizeof(struct sockaddr_in);

        // Accept a connection from the specified socket and write the client
        // address and its size to client_addr and sin_size; return the socket
        // to use for communicating to the client.
        new_sockfd = accept(sockfd, (struct sockaddr *)&client_addr, &sin_size);
        if (-1 == new_sockfd)
            fatal("server: fatal error while accepting connection!");

        printf("server: got connection from %s port %d\n",
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // Send a message to the client to indicate that the connection was
        // established.
        char *msg = "Hello, Client!";
        if (-1 == send(new_sockfd, msg, strlen(msg), 0)) {
            fatal("server: fatal error while sending message!");
        }
        printf("server: message sent: %s\n", msg);

        // Receive messages from client as long as they have non-zero length.
        recv_length = recv(new_sockfd, &buffer, 1024, 0);
        while (recv_length > 0) {
            printf("server: received message: %s\n", (unsigned char *)&buffer);
            recv_length = recv(new_sockfd, &buffer, 1024, 0);
        }

        printf("server: disconnected from %s port %d\n",
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        close(new_sockfd);
    }
    return 0;
}
