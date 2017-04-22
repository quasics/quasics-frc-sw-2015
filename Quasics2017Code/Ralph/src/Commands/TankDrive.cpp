#include "TankDrive.h"

#include "../Robot.h"
#include "../RobotVariables.h"

TankDrive::TankDrive() {
	Requires(Robot::driveTrain.get());
	lastButton = false;
	isInverted = false;
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
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

	if (lastButton && !Robot::oi->getDriveStick()->GetRawButton(XButton))
		isInverted = !isInverted;

	if (!isInverted) {
		Robot::driveTrain->SetLeftPower(-leftStick);
		Robot::driveTrain->SetRightPower(-rightStick);
	} else {
		Robot::driveTrain->SetLeftPower(rightStick);
		Robot::driveTrain->SetRightPower(leftStick);
	}

	lastButton = Robot::oi->getDriveStick()->GetRawButton(XButton);
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
