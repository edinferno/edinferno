/// Defines structures representing time.
#ifndef EDANTE_TIME_H
#define EDANTE_TIME_H

#include <stdint.h>

/// Specifies a point in 1D time.
class Timestamp {
public:
    /// Returns the timestamp value.
    uint64_t getValue();
};

#endif
