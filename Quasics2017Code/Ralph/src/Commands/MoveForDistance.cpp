#include "MoveForDistance.h"

MoveForDistance::MoveForDistance(int targetInches, float powerMagnitude) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
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

	Robot::driveTrain->SetLeftPower(power);
	Robot::driveTrain->SetRightPower(power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDistance::Execute() {
	if(fabs(Robot::driveTrain->LeftEncoderDistance()) >= fabs (target))
		Robot::driveTrain->SetLeftPower(0);
	if(fabs(Robot::driveTrain->RightEncoderDistance()) >= fabs (target))
			Robot::driveTrain->SetRightPower(0);
};

// Make this return true when this Command no longer needs to run execute()
bool MoveForDistance::IsFinished() {
	return (fabs(Robot::driveTrain->LeftEncoderDistance()) >= fabs (target) && fabs(Robot::driveTrain->RightEncoderDistance()) >= fabs (target));
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
