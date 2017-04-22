#include "GearDoorTeleop.h"

GearDoorTeleop::GearDoorTeleop() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::gearSystem.get());
	previousButton = false;	//xButtonX
}

// Called just before this Command runs the first time
void GearDoorTeleop::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void GearDoorTeleop::Execute() {
	if (previousButton && !Robot::oi->getAuxStick()->GetRawButton(bButtonX)) {
		if(Robot::gearSystem->Get()){
			Robot::gearSystem->Set(false);
		} else {
			Robot::gearSystem->Set(true);
		}
	}
	previousButton = Robot::oi->getAuxStick()->GetRawButton(bButtonX);
}

// Make this return true when this Command no longer needs to run execute()
bool GearDoorTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GearDoorTeleop::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearDoorTeleop::Interrupted() {

}
