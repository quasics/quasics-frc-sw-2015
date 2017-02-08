#include "ClimberAuto.h"
#include "WPILib.h"
#include "../Robot.h"

ClimberAuto::ClimberAuto(double power) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::climber.get());
	powerPercent = power;
}

// Called just before this Command runs the first time
void ClimberAuto::Initialize() {
	Robot::climber->TurnOn(powerPercent);
}

// Called repeatedly when this Command is scheduled to run

// Make this return true when this Command no longer needs to run execute()
bool ClimberAuto::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ClimberAuto::End() {
	Robot::climber->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimberAuto::Interrupted() {
	Robot::climber->TurnOff();
}
