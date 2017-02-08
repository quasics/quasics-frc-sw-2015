#include "ThreeSecondIntake.h"

#include "../Robot.h"
#include "../RobotMap.h"

ThreeSecondIntake::ThreeSecondIntake(double seconds, double power) {
	powerPercent = power;
	m_seconds = seconds;
	counter = 0;
	Requires(Robot::intake.get());
}

// Called just before this Command runs the first time
void ThreeSecondIntake::Initialize() {
	Robot::intake->TurnOn(powerPercent);
	counter = 0;
}

// Called repeatedly when this Command is scheduled to run
void ThreeSecondIntake::Execute() {
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool ThreeSecondIntake::IsFinished() {
    return counter >= m_seconds * 50;
}

// Called once after isFinished returns true
void ThreeSecondIntake::End() {
	Robot::intake->TurnOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ThreeSecondIntake::Interrupted() {
	Robot::intake->TurnOff();
}
