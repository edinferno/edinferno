/// Defines structures representing space.
#ifndef EDANTE_SPACE_H
#define EDANTE_SPACE_H

/// Specifies a point in 2D space.
struct Point2D {
    double x;
    double y;
};


/// Specifies a vector in 2D space.
struct Vector2D {
    double dx;
    double dy;
};

/// Specifies a point and orientation in 2D space.
struct Pose2D {
    double x;
    double y;
    double theta;
};

#endif
