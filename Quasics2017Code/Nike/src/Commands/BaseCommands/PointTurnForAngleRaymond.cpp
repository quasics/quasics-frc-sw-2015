#include <Commands/BaseCommands/PointTurnForAngleRaymond.h>

PointTurnForAngle::PointTurnForAngle(double targetDegreesAntiClockwise,
		double power) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
	userPower = std::max(fabs(power), .2);
	targetDegrees = targetDegreesAntiClockwise;
	isAntiClockwise = false;

	if (targetDegreesAntiClockwise > 0)
		isAntiClockwise = true;
	else
		isAntiClockwise = false;

	targetDegrees = int(fabs(targetDegrees)+.5) % 360;
	if (targetDegrees >= 180) {
		isAntiClockwise = !isAntiClockwise;
		targetDegrees = targetDegrees - 180;
	}
}

// Called just before this Command runs the first time
void PointTurnForAngle::Initialize() {
	Robot::driveBase->ResetYaw();
	if (isAntiClockwise){
		Robot::driveBase->SetLeftPower(-userPower);
		Robot::driveBase->SetRightPower(userPower);
	} else {
		Robot::driveBase->SetLeftPower(userPower);
		Robot::driveBase->SetRightPower(-userPower);
	}
	SmartDashboard::PutNumber("Target Angle", targetDegrees);
}

// Called repeatedly when this Command is scheduled to run
void PointTurnForAngle::Execute() {
	if (targetDegrees - fabs(Robot::driveBase->ReadCurrentHeading()) < 45){
		if (isAntiClockwise){
			Robot::driveBase->SetLeftPower(-.1);
			Robot::driveBase->SetRightPower(.1);
		} else {
			Robot::driveBase->SetLeftPower(.1);
			Robot::driveBase->SetRightPower(-.1);
		}
	}
	SmartDashboard::PutNumber("Current Angle", Robot::driveBase->ReadCurrentHeading());
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnForAngle::IsFinished() {
	return fabs(Robot::driveBase->ReadCurrentHeading()) >= targetDegrees;
}

// Called once after isFinished returns true
void PointTurnForAngle::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnForAngle::Interrupted() {
	Robot::driveBase->Stop();
}
