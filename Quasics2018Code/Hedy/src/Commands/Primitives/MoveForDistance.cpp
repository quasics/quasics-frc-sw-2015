#include "MoveForDistance.h"
#include "../../Robot.h"

MoveForDistance::MoveForDistance(uint32_t targetInches, double powerLevel)
: frc::Command(), target(targetInches), power(powerLevel) {
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void MoveForDistance::Initialize() {
	Robot::driveBase->RightEncoderReset();
	Robot::driveBase->LeftEncoderReset();
	Robot::driveBase->SetPowerToMotors(-(power), power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDistance::Execute() {
	if(Robot::driveBase->LeftEncoderDistance()  > Robot::driveBase->RightEncoderDistance()){
		Robot::driveBase->SetPowerToMotors(-(power - .1), (power + .3));
	}
	else if(Robot::driveBase->LeftEncoderDistance() < Robot::driveBase->RightEncoderDistance()){
		Robot::driveBase->SetPowerToMotors(-(power + .1), (power -.1));
	}
	else{
		Robot::driveBase->SetPowerToMotors(-(power), power);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForDistance::IsFinished() {
	return (fabs(Robot::driveBase->LeftEncoderDistance()) > target
			|| fabs(Robot::driveBase->RightEncoderDistance()) > target);
}

// Called once after isFinished returns true
void MoveForDistance::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDistance::Interrupted() {
	End();
}
