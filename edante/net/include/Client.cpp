#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "Networking.h"
//#include "SPLStandardMessage.h"

void Networking::sendStandardMessage(SPLStandardMessage splMsg) {
    int sockfd;
    struct sockaddr_in target_addr;
    struct in_addr     target_in_addr; // 127.0.0.1
    uint16_t           target_in_port; //7890

    inet_aton("127.0.0.1", &target_in_addr);
    target_in_port = atoi("7890");

    // Fill the target_addr struct
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_in_port);
    target_addr.sin_addr = target_in_addr;
    memset(&(target_addr.sin_zero), '\0', 8);

    sockfd = socket(PF_INET, SOCK_DGRAM, 0);

    connect(sockfd, (struct sockaddr *) &target_addr, sizeof(struct sockaddr));

    send(sockfd, &splMsg, sizeof(splMsg), 0);
}