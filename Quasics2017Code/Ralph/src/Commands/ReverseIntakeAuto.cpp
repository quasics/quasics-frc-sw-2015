#include "ReverseIntakeAuto.h"
#include "../Robot.h"

ReverseIntakeAuto::ReverseIntakeAuto(double power) : powerPercent(power) {
	Requires(Robot::intake.get());
}

// Called just before this Command runs the first time
void ReverseIntakeAuto::Initialize() {
	Robot::intake->TurnOn(powerPercent);
}

// Make this return true when this Command no longer needs to run execute()
bool ReverseIntakeAuto::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReverseIntakeAuto::End() {
	Robot::intake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReverseIntakeAuto::Interrupted() {
	Robot::intake->TurnOff();
}
