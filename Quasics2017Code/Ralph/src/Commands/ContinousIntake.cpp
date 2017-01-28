#include "ContinousIntake.h"
#include "../Robot.h"
ContinousIntake::ContinousIntake(double power) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::intake.get());
	powerPercent = power;
	isMotorOn = false;
	buttonDown = false;
}

// Called just before this Command runs the first time
void ContinousIntake::Initialize() {
	isMotorOn = false;
}

// Called repeatedly when this Command is scheduled to run
void ContinousIntake::Execute() {
	bool buttonDown = Robot::oi->getDriveStick()->GetRawButton(5);
	if (buttonDown && isMotorOn){
		// Take appropriate action
		Robot::intake->TurnOff();
		isMotorOn = false;
	} else if (!buttonDown && !isMotorOn) {
		// Take appropriate action
		Robot::intake->TurnOn(powerPercent);
		isMotorOn = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ContinousIntake::IsFinished() {
	// TODO: Replace this function with intentional code.
	return false;
}

// Called once after isFinished returns true
void ContinousIntake::End() {
	Robot::intake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run

// Called once after isFinished returns true
void ContinousIntake::Interrupted() {
	Robot::intake->TurnOff();

}
