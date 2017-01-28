#include "TankDrive.h"

TankDrive::TankDrive() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void TankDrive::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
	float leftStick;
	float rightStick;

	leftStick = Robot::oi->getDriveStick()->GetRawAxis(1);
	rightStick = Robot::oi->getDriveStick()->GetRawAxis(3);

	Robot::driveTrain->SetLeftPower(leftStick);
	Robot::driveTrain->SetRightPower(rightStick);
}

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TankDrive::End() {
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TankDrive::Interrupted() {
	Robot::driveTrain->Stop();
}
