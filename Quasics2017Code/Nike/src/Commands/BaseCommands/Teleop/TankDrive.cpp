#include "TankDrive.h"

TankDrive::TankDrive() {
	Requires(Robot::driveBase.get());
	lastButton = false;
	isReversed = false;
}

// Called just before this Command runs the first time
void TankDrive::Initialize() {

}

#define MediumMultiplier .6
#define SlowMultiplier .25
#define TurboMultiplier .7

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
	double multiplier = MediumMultiplier;
	if ((Robot::oi->getDriveStick()->GetRawButton(LeftShoulder)
			|| Robot::oi->getDriveStick()->GetRawButton(RightShoulder))
			&& !(Robot::oi->getDriveStick()->GetRawButton(LeftTrigger)
					|| Robot::oi->getDriveStick()->GetRawButton(RightTrigger))) {
		multiplier = SlowMultiplier;
	} else if (!(Robot::oi->getDriveStick()->GetRawButton(LeftShoulder)
			|| Robot::oi->getDriveStick()->GetRawButton(RightShoulder))
			&& (Robot::oi->getDriveStick()->GetRawButton(LeftTrigger)
					|| Robot::oi->getDriveStick()->GetRawButton(RightTrigger))) {
		multiplier = TurboMultiplier;
	} else {
		multiplier = MediumMultiplier;
	}

	float leftStick = Robot::oi->getDriveStick()->GetRawAxis(LeftYAxis)
			* multiplier;
	float rightStick = Robot::oi->getDriveStick()->GetRawAxis(RightYAxis)
			* multiplier;

	if (lastButton && !Robot::oi->getDriveStick()->GetRawButton(XButton)) {
		isReversed = !isReversed;
	}

	if (isReversed) {
		Robot::driveBase->SetLeftPower(-rightStick);
		Robot::driveBase->SetRightPower(-leftStick);
	} else {
		Robot::driveBase->SetLeftPower(leftStick);
		Robot::driveBase->SetRightPower(rightStick);
	}

	lastButton = Robot::oi->getDriveStick()->GetRawButton(XButton);
}

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TankDrive::End() {
	Robot::driveBase->SetLeftPower();
	Robot::driveBase->SetRightPower();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TankDrive::Interrupted() {
	Robot::driveBase->SetLeftPower();
	Robot::driveBase->SetRightPower();
}
