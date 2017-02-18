#include "DriveBase.h"
#include "../RobotMap.h"

DriveBase::DriveBase() : Subsystem("DriveBase") {
    leftFrontMotor = RobotMap::driveBaseLeftFrontMotor;
    leftRearMotor = RobotMap::driveBaseLeftRearMotor;
    rightFrontMotor = RobotMap::driveBaseRightFrontMotor;
    rightRearMotor = RobotMap::driveBaseRightRearMotor;
    driveStick = RobotMap::driverStick;

#ifdef USE_FRC_DRIVE_CODE
    robotDrive21 = RobotMap::driveBaseRobotDrive21;
#endif	// USE_FRC_DRIVE_CODE
}

void DriveBase::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

#ifndef USE_FRC_DRIVE_CODE
void DriveBase::setLeftPower(float value) {
	leftFrontMotor->Set(value);
	leftRearMotor->Set(value);
}
void DriveBase::setRightPower(float value) {
	rightFrontMotor->Set(value);
	rightRearMotor->Set(value);
}
#endif	// USE_FRC_DRIVE_CODE


// Put methods for controlling this subsystem
// here. Call these from Commands.
const int leftYAxis = 1;
const int rightYAxis = 3;

void DriveBase::tankDrive() {
#ifdef USE_FRC_DRIVE_CODE
	robotDrive21->TankDrive(driveStick.get(), leftYAxis, driveStick.get(), rightYAxis);
#else
	// TODO(healym): Read the multiplier from the driver station (or buttons on the driver stick).
	float multiplier = 1.0f;
	const float leftValue = driveStick->GetRawAxis(leftYAxis) * multiplier;
	const float rightValue = driveStick->GetRawAxis(rightYAxis) * multiplier;
	setLeftPower(leftValue);
	setRightPower(rightValue);
#endif
}
