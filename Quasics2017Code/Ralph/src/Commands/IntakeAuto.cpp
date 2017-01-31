#include "IntakeAuto.h"
#include "../Robot.h"
IntakeAuto::IntakeAuto(double power) {
	Requires(Robot::intake.get());
		powerPercent = power;

}

// Called just before this Command runs the first time
void IntakeAuto::Initialize() {

	Robot::intake->TurnOn(powerPercent);

}

// Make this return true when this Command no longer needs to run execute()
bool IntakeAuto::IsFinished() {

	return false;

}

// Called once after isFinished returns true
void IntakeAuto::End() {

	Robot::intake->TurnOff();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeAuto::Interrupted() {

	Robot::intake->TurnOff();

}
