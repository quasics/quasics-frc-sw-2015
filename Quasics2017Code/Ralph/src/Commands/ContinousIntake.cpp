#include "ContinousIntake.h"
#include "../Robot.h"
#include "../RobotVariables.h"

ContinousIntake::ContinousIntake(double power) {
	Requires(Robot::intake.get());

	// CODE_REVIEW(mjh): This adjustment (/2) should probably be applied at the subsystem level,
	// rather than in an individual command, especially since it's likely to be needed in
	// *any* command that interacts with the intake motors.  (And if we swap out the motor
	// or change the underlying mechanism, we just want to change this in one place -- the
	// subsystem -- rather than in multiple independent commands.)
	powerPercent = power / 2; //The 775 pro runs really fast, so we are going to need to slow it down
	isMotorOn = false;
	buttonDown = false;
}

void ContinousIntake::Initialize() {
	isMotorOn = false;
}

void ContinousIntake::Execute() {
	bool buttonDown = Robot::oi->getDriveStick()->GetRawButton(IntakeButton);
	if (buttonDown && isMotorOn) {
		// Take appropriate action
		Robot::intake->TurnOff();
		isMotorOn = false;
	} else if (!buttonDown && !isMotorOn) {
		// Take appropriate action
		Robot::intake->TurnOn(powerPercent);
		isMotorOn = true;
	}
}

bool ContinousIntake::IsFinished() {
	return false;
}

void ContinousIntake::End() {
	Robot::intake->TurnOff();
}

void ContinousIntake::Interrupted() {
	Robot::intake->TurnOff();
}
