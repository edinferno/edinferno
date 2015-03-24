#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "network.h"

int main(int argc, char *argv[]) {
    int sockfd;
    struct sockaddr_in target_addr;
    struct in_addr     target_in_addr;
    uint16_t           target_in_port;

    int recv_length;

    unsigned char buffer[1024];

    if (argc < 3) {
        printf("Usage: %s <address> <port>\n", argv[0]);
        exit(1);
    }

    // Parse address from command line arguments
    inet_aton(argv[1], &target_in_addr);
    target_in_port = atoi(argv[2]);

    // Fill the target_addr struct
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_in_port);
    target_addr.sin_addr = target_in_addr;
    memset(&(target_addr.sin_zero), '\0', 8);  // Zero the rest of the struct

    printf("client: Initializing socket...\n");

    if ((sockfd = socket(PF_INET, SOCK_DGRAM, 0)) == -1)
        fatal("client: fatal error in socket!");

    printf("client: Default UDP target: %s:%d...\n", argv[1], target_in_port);

    if (-1 == connect(sockfd,
                      (struct sockaddr *)&target_addr,
                      sizeof(struct sockaddr))) {
        fatal("client: fatal error while setting target UDP target!");
    }

    printf("client: Sending message...\n");

    unsigned char *msg = "Hello, Server!";
    if (-1 == send(sockfd, msg, strlen(msg), 0)) {
        fatal("client: fatal error while sending message!");
    }
    printf("client: Message sent: [%s].\n", (char *)msg);

    return 0;
}
