#include <iostream>
#include "Communication.h"
#include "Space.h"
#include "Mental.h"

using namespace std;

int main() {
	Communication comms;

	Timestamp dummyTimestamp{0};
	Pose2D dummyPose{0.2f, 0.4f, 0.8f};
	Point2D dummyWalk{0.3f, 0.5f};
	Point2D dummyShoot{0.5f, 0.7f};
	Point2D dummyBallPos{0.6f, 0.8f};
	Vector2D dummyBallVel{0.7f, 3.0f};
	Intention dummyIntetion{Intention::FIND_ONESELF};

	comms.setPose(dummyPose, dummyTimestamp);
	comms.setWalkingTo(dummyWalk, dummyTimestamp);
	comms.setShootingTo(dummyShoot, dummyTimestamp);
	comms.setBallPosition(dummyBallPos, dummyTimestamp);
	comms.setBallVelocity(dummyBallVel, dummyTimestamp);
	comms.setIntention(dummyIntetion, dummyTimestamp);

	SPLStandardMessage standardMessage = comms.getStandardMessage();

	cout << standardMessage << endl;
	return 0;
}