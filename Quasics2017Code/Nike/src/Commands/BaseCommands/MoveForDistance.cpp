#include <Commands/BaseCommands/MoveForDistance.h>

MoveForDistance::MoveForDistance(uint32_t targetInches, double powerLevel) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
	target = targetInches;
	power = powerLevel;
}

// Called just before this Command runs the first time
void MoveForDistance::Initialize() {
	Robot::driveBase->RightEncoderReset();
	Robot::driveBase->LeftEncoderReset();
	Robot::driveBase->SetLeftPower(power);
	Robot::driveBase->SetRightPower(power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDistance::Execute() {
	if(power > .15 && target - Robot::driveBase->LeftEncoderDistance() <= 24){
		Robot::driveBase->SetLeftPower(.15);
		Robot::driveBase->SetRightPower(.15);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForDistance::IsFinished() {
	return (fabs(Robot::driveBase->LeftEncoderDistance()) > target
			|| fabs(Robot::driveBase->LeftEncoderDistance()) > target);
}

// Called once after isFinished returns true
void MoveForDistance::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDistance::Interrupted() {
	Robot::driveBase->Stop();
}
