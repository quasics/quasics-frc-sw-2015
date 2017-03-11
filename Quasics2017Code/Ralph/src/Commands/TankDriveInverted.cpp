#include "TankDriveInverted.h"
#include "../Robot.h"
#include "../RobotVariables.h"

TankDriveInverted::TankDriveInverted() {
	Requires(Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void TankDriveInverted::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TankDriveInverted::Execute() {
	double multiplier = MediumMultiplier;
	if ((Robot::oi->getDriveStick()->GetRawButton(LeftShoulder)
			|| Robot::oi->getDriveStick()->GetRawButton(RightShoulder))
			&& !(Robot::oi->getDriveStick()->GetRawButton(LeftTrigger)
					|| Robot::oi->getDriveStick()->GetRawButton(RightTrigger)))
		multiplier = SlowMultiplier;
	else if (!(Robot::oi->getDriveStick()->GetRawButton(LeftShoulder)
			|| Robot::oi->getDriveStick()->GetRawButton(RightShoulder))
			&& (Robot::oi->getDriveStick()->GetRawButton(LeftTrigger)
					|| Robot::oi->getDriveStick()->GetRawButton(RightTrigger)))
		multiplier = TurboMultiplier;
	else
		multiplier = MediumMultiplier;

	float leftStick = -Robot::oi->getDriveStick()->GetRawAxis(LeftYAxis)
			* multiplier;
	float rightStick = -Robot::oi->getDriveStick()->GetRawAxis(RightYAxis)
			* multiplier;

#ifdef USE_TANK_DRIVE_TRIM
	Robot::driveTrain->SetTrimmedPower(leftStick, rightStick);

#else
	Robot::driveTrain->SetLeftPower(rightStick * 1.2);
	Robot::driveTrain->SetRightPower(leftStick * .7);
#endif
}

// Make this return true when this Command no longer needs to run execute()
bool TankDriveInverted::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TankDriveInverted::End() {
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TankDriveInverted::Interrupted() {
	Robot::driveTrain->Stop();
}
