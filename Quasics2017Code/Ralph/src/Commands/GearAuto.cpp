#include "GearAuto.h"
#include "../Robot.h"


GearAuto::GearAuto(bool doorOpen) {
	Requires(Robot::gear.get());
	openDoor = doorOpen;
}

void GearAuto::Initialize() {
	Robot::gear->Set(openDoor);
}

// Make this return true when this Command no longer needs to run execute()
bool GearAuto::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void GearAuto::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearAuto::Interrupted() {

}
