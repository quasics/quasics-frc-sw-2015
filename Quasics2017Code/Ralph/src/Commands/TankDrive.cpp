#include "TankDrive.h"
#include "math.h"

TankDrive::TankDrive() {
	Requires(Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void TankDrive::Initialize() {

}

//TODO: Add in turbo and turtle modes
// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
	float leftStick = Robot::oi->getDriveStick()->GetRawAxis(LeftYAxis);
	float rightStick = Robot::oi->getDriveStick()->GetRawAxis(RightYAxis);

	float leftPower = 0;
	float rightPower = 0;

#ifdef USE_TANK_DRIVE_TRIM_
	float leftMaxRate = Robot::driveTrain->LeftEncoderVelocity()/leftStick;
	float rightMaxRate = Robot::driveTrain->RightEncoderVelocity()/rightStick;


#else
	leftPower = leftStick;
	rightPower = rightStick;
#endif
	Robot::driveTrain->SetLeftPower(leftPower);
	Robot::driveTrain->SetRightPower(rightPower);
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
