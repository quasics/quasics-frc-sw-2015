#include <Commands/MoveForward.h>

MoveForward::MoveForward(double powerLevel, double seconds) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires (Robot::driveBase.get());
	power = powerLevel;
	sec = seconds;
}

// Called just before this Command runs the first time
void MoveForward::Initialize() {
	Robot::driveBase->SetLeftPower(0);
	Robot::driveBase->SetRightPower(0);
	counter = 0;
}

// Called repeatedly when this Command is scheduled to run
void MoveForward::Execute() {
	if (counter < (sec * 50) - 10){
		Robot::driveBase->SetLeftPower(power);
		Robot::driveBase->SetRightPower(power);
	}
	else{
		Robot::driveBase->SetLeftPower(power / 2);
		Robot::driveBase->SetRightPower(power / 2);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForward::IsFinished() {
	return counter >= sec;
}

// Called once after isFinished returns true
void MoveForward::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForward::Interrupted() {
	Robot::driveBase->Stop();
}
