#ifndef PIPPI_NETWORK_M
#define PIPPI_NETWORK_M
#include <stdio.h>
#include <stdlib.h>

void fatal(char const * const msg) {
    fprintf(stderr, "%s\n", msg);
    exit(0);
}

void dump(const unsigned char *data_buffer, const unsigned int length) {
    unsigned char byte;
    unsigned int i, j;
    for (i = 0; i < length; ++i) {
        byte = data_buffer[i];
        printf("%02x ", byte);  // Display byte in hex.
        // Every sixteen bytes, or at the end of the buffer:
        if (((i % 16) == 15) || (i == length - 1)) {
            // Add padding at the end of the last line
            for (j = 0; j < 15 - (i % 16); ++j) {
                printf("   ");
            }
            printf("| ");
            for (j = (i - (i % 16)); j <= i; ++j) {  // Display printable bytes
                byte = data_buffer[j];
                if ((byte > 31) && (byte < 127)) {
                    printf("%c", byte);
                } else {
                    printf(".");
                }
            }
            printf("\n");  // End of the dump line
        }
    }
}

#endif
