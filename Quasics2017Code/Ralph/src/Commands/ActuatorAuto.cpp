#include "ActuatorAuto.h"
#include "../Robot.h"

ActuatorAuto::ActuatorAuto(bool doorOpen) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::fuelExhaustGate.get());
	openDoor = doorOpen;
}

// Called just before this Command runs the first time
void ActuatorAuto::Initialize() {
	Robot::fuelExhaustGate->Set(openDoor);
}


// Make this return true when this Command no longer needs to run execute()
bool ActuatorAuto::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void ActuatorAuto::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ActuatorAuto::Interrupted() {

}
