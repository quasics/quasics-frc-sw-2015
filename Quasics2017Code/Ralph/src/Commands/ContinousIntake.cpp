#include "ContinousIntake.h"
#include "../Robot.h"
#include "RobotVariables.h"
ContinousIntake::ContinousIntake(double power) {
	Requires(Robot::intake.get());
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
