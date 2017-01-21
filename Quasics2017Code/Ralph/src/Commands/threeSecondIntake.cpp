#include "threeSecondIntake.h"
#include "../RobotMap.h"

threeSecondIntake::threeSecondIntake(double seconds, double power) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	power = powerPercent;
	seconds = m_seconds;
	counter = 0;
	Requires(Robot::intake.get());
}

// Called just before this Command runs the first time
void threeSecondIntake::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void threeSecondIntake::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool threeSecondIntake::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void threeSecondIntake::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void threeSecondIntake::Interrupted() {

}
