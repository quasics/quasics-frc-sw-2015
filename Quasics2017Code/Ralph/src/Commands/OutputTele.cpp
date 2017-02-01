#include "OutputTele.h"

OutputTele::OutputTele(double power) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	powerPercent = power;
	motorOn = false;
	buttonDown = false;
}

// Called just before this Command runs the first time
void OutputTele::Initialize() {
	motorOn = false;
}

// Called repeatedly when this Command is scheduled to run
void OutputTele::Execute() {
	buttonDown = Robot::oi->getDriveStick()->GetRawButton(6);
	if (buttonDown && motorOn){
		Robot::outtake->TurnOff();
		motorOn = false;
	}
	else if(!buttonDown && !motorOn){
		Robot::outtake->TurnOn(powerPercent);
		motorOn = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool OutputTele::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void OutputTele::End() {
	Robot::outtake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OutputTele::Interrupted() {
	Robot::outtake->TurnOff();
}
