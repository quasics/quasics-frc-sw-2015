#include "GearDoorTeleop.h"

GearDoorTeleop::GearDoorTeleop() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::gearSystem.get());
	previousButton = false;	//xButtonX
	previousKickerButton = false;
}

// Called just before this Command runs the first time
void GearDoorTeleop::Initialize() {
	previousButton = false;
	previousKickerButton = false;
}

// Called repeatedly when this Command is scheduled to run
void GearDoorTeleop::Execute() {
	if (previousButton && !Robot::oi->getAuxStick()->GetRawButton(bButtonX)) {
		if (Robot::gearSystem->Get()) {
			Robot::gearSystem->Set(false);
		} else {
			Robot::gearSystem->Set(true);
		}
	}

	if (previousKickerButton && !Robot::oi->getAuxStick()->GetRawButton(xButtonX)) {
		if (Robot::gearSystem->GetKicker()) {
			Robot::gearSystem->SetKicker(false);
		} else {
			Robot::gearSystem->SetKicker(true);
		}
	}
	previousButton = Robot::oi->getAuxStick()->GetRawButton(bButtonX);
	previousKickerButton = Robot::oi->getAuxStick()->GetRawButton(xButtonX);
}

// Make this return true when this Command no longer needs to run execute()
bool GearDoorTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GearDoorTeleop::End() {
	Robot::gearSystem->SetKicker(false);
	Robot::gearSystem->Set(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearDoorTeleop::Interrupted() {
	Robot::gearSystem->SetKicker(false);
	Robot::gearSystem->Set(false);
}
