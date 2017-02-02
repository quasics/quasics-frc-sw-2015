#include "ContinousIntake.h"
#include "../Robot.h"
#include "RobotVariables.h"
ContinousIntake::ContinousIntake(double power) {
	Requires(Robot::intake.get());
	powerPercent = power;
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
