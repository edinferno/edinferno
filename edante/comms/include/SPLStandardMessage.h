#ifndef SPLSTANDARDMESSAGE_H
#define SPLSTANDARDMESSAGE_H

#include <stdint.h>
#include <iomanip>

#define SPL_STANDARD_MESSAGE_STRUCT_HEADER  "SPL "
#define SPL_STANDARD_MESSAGE_STRUCT_VERSION 6
#define SPL_STANDARD_MESSAGE_DATA_SIZE      780
#define SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 5

/*
 Important remarks about units:

 For each parameter, the respective comments describe its unit.
 The following units are used:
 
   - Distances:  Millimeters (mm)
   - Angles:     Radian
   - Time:       Seconds (s)
   - Speed:      Millimeters per second (mm/s)
*/


struct SPLStandardMessage 
{
  char header[4];        // "SPL "
  uint8_t version;       // has to be set to SPL_STANDARD_MESSAGE_STRUCT_VERSION
  int8_t playerNum;      // [MANDATORY FIELD] 1-5
  int8_t teamNum;        // [MANDATORY FIELD] the number of the team (as provided by the organizers)
  int8_t fallen;         // [MANDATORY FIELD] 1 means that the robot is fallen, 0 means that the robot can play

  // [MANDATORY FIELD]
  // position and orientation of robot
  // coordinates in millimeters
  // 0,0 is in center of field
  // +ve x-axis points towards the goal we are attempting to score on
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  // angle in radians, 0 along the +x axis, increasing counter clockwise
  float pose[3];      // x,y,theta
  
  // [MANDATORY FIELD]
  // the robot's target position on the field
  // the coordinate system is the same as for the pose
  // if the robot does not have any target, this attribute should be set to the robot's position
  float walkingTo[2]; 
  
  // [MANDATORY FIELD]
  // the target position of the next shot (either pass or goal shot)
  // the coordinate system is the same as for the pose
  // if the robot does not intend to shoot, this attribute should be set to the robot's position
  float shootingTo[2]; 

  // ball information
  float ballAge;        // seconds since this robot last saw the ball. -1.f if we haven't seen it

  // position of ball relative to the robot
  // coordinates in millimeters
  // 0,0 is in center of the robot
  // +ve x-axis points forward from the robot
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  float ball[2];

  // velocity of the ball (same coordinate system as above)
  // the unit is millimeters per second
  float ballVel[2];
  
  // describes what - in the robot's opinion - the teammates should do:
  // 0 - nothing particular (default)
  // 1 - play keeper
  // 2 - support defense
  // 3 - support offense
  // 4 - play the ball
  // For each teammate, the corresponding suggestion is put in the element
  // playerNumber(teammate) -1.
  // Example: To suggest a teammate, which has the number 5, to play in defense:
  //          suggestion[4] = 2;
  int8_t suggestion[SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];
  
  // [MANDATORY FIELD]
  // describes what the robot intends to do:
  // 0 - nothing particular (default)
  // 1 - wants to be keeper
  // 2 - wants to play defense
  // 3 - wants to play the ball
  // 4 - robot is lost
  int8_t intention;
  
  // [MANDATORY]
  // the average speed that the robot has, for instance, when walking towards the ball
  // the unit is mm/s
  // the idea of this value is to roughly represent the robot's walking skill
  // it has to be set once at the beginning of the game and remains fixed
  int16_t averageWalkSpeed;
  
  // [MANDATORY]
  // the maximum distance that the ball rolls after a strong kick by the robot
  // the unit is mm
  // the idea of this value is to roughly represent the robot's kicking skill
  // it has to be set once at the beginning of the game and remains fixed
  int16_t maxKickDistance;

  // [MANDATORY]
  // describes the current confidence of a robot about its self-location,
  // the unit is percent [0,..100]
  // the value should be updated in the course of the game
  int8_t currentPositionConfidence;

  // [MANDATORY]
  // describes the current confidence of a robot about playing in the right direction,
  // the unit is percent [0,..100]
  // the value should be updated in the course of the game
  int8_t currentSideConfidence;
  
  // number of bytes that is actually used by the data array
  uint16_t numOfDataBytes;

  // buffer for arbitrary data, teams do not need to send more than specified in numOfDataBytes
  uint8_t data[SPL_STANDARD_MESSAGE_DATA_SIZE];

#ifdef __cplusplus
  // constructor
  SPLStandardMessage() :
    version(SPL_STANDARD_MESSAGE_STRUCT_VERSION),
    playerNum(-1),
    teamNum(-1),
    fallen(-1),
    ballAge(-1.f),
    intention(-1),
    averageWalkSpeed(-1),
    maxKickDistance(-1),
    currentPositionConfidence(-1),
    currentSideConfidence(-1),
    numOfDataBytes(0)
  {
    const char* c = SPL_STANDARD_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = c[i];
    pose[0] = 0.f;
    pose[1] = 0.f;
    pose[2] = 0.f;
    walkingTo[0] = 0.f;
    walkingTo[1] = 0.f;
    shootingTo[0] = 0.f;
    shootingTo[1] = 0.f;
    ball[0] = 0.f;
    ball[1] = 0.f;
    ballVel[0] = 0.f;
    ballVel[1] = 0.f;
    for(int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      suggestion[i] = -1;
  }

  friend std::ostream& operator<<(std::ostream& stream, const SPLStandardMessage& obj) {
    stream << "Pose: " << obj.pose[0] << " " << obj.pose[1] << " " << obj.pose[2] << std::endl;
    stream << "Walking To: " << obj.walkingTo[0] << " " << obj.walkingTo[1] << std::endl;
    stream << "Shooting To: " << obj.shootingTo[0] << " " << obj.shootingTo[1] << std::endl;

    stream << "Ball Age: " << obj.ballAge << std::endl;
    stream << "Ball Position: " << obj.ball[0] << " " << obj.ball[1] << std::endl;
    stream << "Ball Velocity: " << obj.ballVel[0] << " " << obj.ballVel[1] << std::endl;

    stream << "Suggestions: "; 
    for (int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; i++) {
        stream << (int)obj.suggestion[i] << " ";
    }
    stream << std::endl;
    stream << "Intention: " << (int)obj.intention << std::endl;

    stream << "Average Walking Speed: " << obj.averageWalkSpeed << std::endl;
    stream << "Max Kick Distance: " << obj.maxKickDistance << std::endl;

    stream << "Current Position Confidence: " << (int)obj.currentPositionConfidence << std::endl;
    stream << "Current Side Confidence: " << (int)obj.currentSideConfidence << std::endl;

    stream << "Number of Data Bytes: " << obj.numOfDataBytes << std::endl;
    stream << "Data: " ;
    for (int i = 0; i < obj.numOfDataBytes; i++) {
      stream << std::hex << (int)obj.data[i] << " ";
    }
    stream << std::endl;
    return stream;
  }
#endif
};

#endif // SPLSTANDARDMESSAGE_H