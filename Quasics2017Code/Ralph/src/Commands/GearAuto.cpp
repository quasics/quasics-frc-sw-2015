#include "GearAuto.h"
#include "../Robot.h"
#include "../Subsystems/Gear.h"

GearAuto::GearAuto(bool doorOpen) {
	Requires(Robot::gear.get());
	openDoor = doorOpen;
	counter = 0;
	if (doorOpen) {
		kickerDelay = true;
		isDone = false;
	} else {
		kickerDelay = false;
		isDone = true;
	}
}

void GearAuto::Initialize() {
	if (openDoor) {
		kickerDelay = true;
		isDone = false;
	} else {
		kickerDelay = false;
		isDone = true;
	}
	Robot::gear->Set(openDoor);
	Robot::gear->SetKicker(false);
	counter = 0;
}

void GearAuto::Execute() {
	if (kickerDelay) {
		if (counter >= 50) {
			Robot::gear->SetKicker(true);
			isDone = true;
		}
		counter++;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GearAuto::IsFinished() {
	return isDone;
}

// Called once after isFinished returns true
void GearAuto::End() {
	kickerDelay = true;
	isDone = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearAuto::Interrupted() {
	kickerDelay = true;
	isDone = false;
}
