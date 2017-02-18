#include "IntakeTeleop.h"

#include "../Robot.h"
#include "../RobotVariables.h"

IntakeTeleop::IntakeTeleop(double power) :
		powerPercent(power), buttonPrev(false), motorOn(false) {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::intake.get());
}

// Called just before this Command runs the first time
void IntakeTeleop::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void IntakeTeleop::Execute() {
	bool buttonDown = Robot::oi->getAuxStick()->GetRawButton(IntakeButton);
	if (!buttonDown && buttonPrev) {
		motorOn = !motorOn;
	}
	if (motorOn) {
		Robot::intake->TurnOn(powerPercent);
	} else {
		Robot::intake->TurnOff();
	}
	buttonPrev = buttonDown;
}

// Make this return true when this Command no longer needs to run execute()
bool IntakeTeleop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void IntakeTeleop::End() {
	Robot::intake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeTeleop::Interrupted() {
	Robot::intake->TurnOff();
}
