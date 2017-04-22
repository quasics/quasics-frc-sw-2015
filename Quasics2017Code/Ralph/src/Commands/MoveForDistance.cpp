#include "MoveForDistance.h"
#include "../Robot.h"
#include <iostream>

MoveForDistance::MoveForDistance(int targetInches, float powerMagnitude) {
	Requires(Robot::driveTrain.get());

	target = targetInches;
	if (target >= 0)
		power = fabs(powerMagnitude);
	else
		power = -fabs(powerMagnitude);

	Robot::driveTrain->LeftEncoderReset();
	Robot::driveTrain->RightEncoderReset();
}

// Called just before this Command runs the first time
void MoveForDistance::Initialize() {
	Robot::driveTrain->LeftEncoderReset();
	Robot::driveTrain->RightEncoderReset();

	Robot::driveTrain->SetLeftPower(power * .8);
	Robot::driveTrain->SetRightPower(power);
}

void MoveForDistance::Execute() {
	std::cout << "\n Target:" << target << "\n LeftEncoder: "
			<< Robot::driveTrain->LeftEncoderDistance()<< "\n RightEncoder: "
			<< Robot::driveTrain->RightEncoderDistance();
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForDistance::IsFinished() {
	return (fabs(Robot::driveTrain->LeftEncoderDistance()) >= fabs(target)
			|| fabs(Robot::driveTrain->RightEncoderDistance()) >= fabs(target));
}

// Called once after isFinished returns true
void MoveForDistance::End() {
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDistance::Interrupted() {
	Robot::driveTrain->Stop();
}
