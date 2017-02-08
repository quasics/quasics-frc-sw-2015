#include "FuelExhaustAuto.h"
#include "../Robot.h"

FuelExhaustAuto::FuelExhaustAuto(bool doorOpen) {
	Requires (Robot::fuelExhaustGate.get());
	openDoor = doorOpen;
}

// Called just before this Command runs the first time
void FuelExhaustAuto::Initialize() {
	Robot::fuelExhaustGate->Set(openDoor);
}

// Make this return true when this Command no longer needs to run execute()
bool FuelExhaustAuto::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void FuelExhaustAuto::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FuelExhaustAuto::Interrupted() {

}
