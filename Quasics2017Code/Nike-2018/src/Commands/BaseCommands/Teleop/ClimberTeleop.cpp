#include "ClimberTeleop.h"

ClimberTeleop::ClimberTeleop() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::climberSystem.get());
	previousButton = false;
	isOn = false;
}

// Called just before this Command runs the first time
void ClimberTeleop::Initialize() {
	Robot::climberSystem->TurnOff();
	previousButton = false;
	isOn = false;
}

// Called repeatedly when this Command is scheduled to run
void ClimberTeleop::Execute() {
	if (!Robot::oi->getAuxStick()->GetRawButton(LeftTriggerX) && previousButton) {
		if (isOn) {
			Robot::climberSystem->TurnOff();
			isOn = false;
		} else {
			Robot::climberSystem->Climb();
			isOn = true;
		}
	}
	previousButton = Robot::oi->getAuxStick()->GetRawButton(LeftTriggerX);
}

// Make this return true when this Command no longer needs to run execute()
bool ClimberTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ClimberTeleop::End() {

	Robot::climberSystem->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimberTeleop::Interrupted() {
	Robot::climberSystem->TurnOff();

}
