/// Defines structures representing mental states.
#ifndef EDANTE_MENTAL_H
#define EDANTE_MENTAL_H

/// Specifies a robot's intention options.
enum class Intention {
    /// This is the default intention of the robot
    NONE            = 0,
    GOAL_KEEPING    = 1,
    DEFENDING       = 2,
    ATTACKING       = 3,
    /// This is the intention of the robot when it is lost
    FIND_ONESELF    = 4
};

#endif
