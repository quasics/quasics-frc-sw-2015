#include "PointTurnForAngle.h"

PointTurnForAngle::PointTurnForAngle(double targetDegreesAntiClockwise, double powerPercent) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	targetDegrees = targetDegreesAntiClockwise;

	// Normalize the heading we want to go to, so that it's in the range
	// of +180 to -180 that we can get from the sensor.
	while (targetDegrees > 180) {
		targetDegrees -= 360;
	}
	while (targetDegrees < -180) {
		targetDegrees += 360;
	}

	power = fabs(powerPercent);
	Requires(Robot::driveTrain.get());
	Requires(Robot::gyro.get());
}

// Called just before this Command runs the first time
void PointTurnForAngle::Initialize() {
	Robot::gyro->Reset();
	if (targetDegrees != 0) {
		if (targetDegrees > 180) {
			Robot::driveTrain->SetLeftPower(power);
			Robot::driveTrain->SetRightPower(-power);
		} else{
			Robot::driveTrain->SetLeftPower(-power);
			Robot::driveTrain->SetRightPower(power);
		}
	}
}

// Called repeatedly when this Command is scheduled to run
void PointTurnForAngle::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnForAngle::IsFinished() {
	Robot::gyro->ReadCurrentHeading();
	if (targetDegrees > 0) {	// We're turning to the right (to a positive heading)
		if (Robot::gyro->ReadCurrentHeading() >= targetDegrees) {
			return true;
		}else {
			return false;
		}
	} else {		// We're turning to the left (to a negative heading)
		if (Robot::gyro->ReadCurrentHeading() <= targetDegrees){
			return true;
		}else {
			return false;
		}
	}
}

// Called once after isFinished returns true
void PointTurnForAngle::End() {
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnForAngle::Interrupted() {
	Robot::driveTrain->Stop();
}
