#include "GearAuto.h"
#include "../Robot.h"


GearAuto::GearAuto(bool doorOpen) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::gear.get());
	openDoor = doorOpen;
}

// Called just before this Command runs the first time
void GearAuto::Initialize() {
	Robot::gear->Set(openDoor);
}

// Called repeatedly when this Command is scheduled to run
void GearAuto::Execute() {

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
