/// Defines structures representing space.
#ifndef EDANTE_SPACE_H
#define EDANTE_SPACE_H

/// Specifies a point in 2D space.
struct Point2D {
    float x;
    float y;
};


/// Specifies a vector in 2D space.
struct Vector2D {
    float dx;
    float dy;
};

/// Specifies a point and orientation in 2D space.
struct Pose2D {
    float x;
    float y;
    float theta;
};

#endif
