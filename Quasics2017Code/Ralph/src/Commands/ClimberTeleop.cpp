#include "ClimberTeleop.h"
#include "../RobotVariables.h"
#include "stdio.h"

ClimberTeleop::ClimberTeleop() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get()
	lastButton = false;
	Requires(Robot::climber.get());
}

// Called just before this Command runs the first time
void ClimberTeleop::Initialize() {
	lastButton = false;
}

// Called repeatedly when this Command is scheduled to run
void ClimberTeleop::Execute() {
	if (!Robot::oi->getAuxStick()->GetRawButton(ClimberButton) && lastButton) {
		if (Robot::climber->GetPower() == 0) {
			Robot::climber->TurnOn(-1);
		} else {
			Robot::climber->TurnOn(0);
		}
	}
	lastButton = Robot::oi->getAuxStick()->GetRawButton(ClimberButton);
}

// Make this return true when this Command no longer needs to run execute()
bool ClimberTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ClimberTeleop::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimberTeleop::Interrupted() {

}
