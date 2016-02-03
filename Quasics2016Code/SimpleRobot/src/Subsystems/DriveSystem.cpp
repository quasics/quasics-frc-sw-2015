// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "DriveSystem.h"
#include "../RobotMap.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

DriveSystem::DriveSystem() :
		Subsystem("DriveSystem") {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	leftFront = RobotMap::driveSystemLeftFront;
	leftRear = RobotMap::driveSystemLeftRear;
	rightFront = RobotMap::driveSystemRightFront;
	rightRear = RobotMap::driveSystemRightRear;
	leftEncoder = RobotMap::driveSystemLeftEncoder;
	rightEncoder = RobotMap::driveSystemRightEncoder;
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	isLeftForward = false;
	isRightForward = false;
	leftPower = 0;
	rightPower = 0;
	try {
		/* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
		/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
		/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
		navx.reset(new AHRS(SPI::Port::kMXP));
	} catch (const std::exception& ex) {
		std::string err_string = "Error instantiating navX-MXP:  ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}
}

void DriveSystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveSystem::MoveLeft(double percentPower) {
	leftPower = percentPower;
	if (percentPower > 0) {
		isLeftForward = true;
	} else {
		isLeftForward = false;
	}
	leftFront->Set(-leftPower / 100);
	leftRear->Set(-leftPower / 100);
}

void DriveSystem::MoveRight(double percentPower) {
	rightPower = percentPower;
	if (percentPower > 0) {
		isRightForward = true;
	} else {
		isRightForward = false;
	}
	rightFront->Set(rightPower / 100);
	rightRear->Set(rightPower / 100);
}

void DriveSystem::StopEverything() {
	leftPower = 0;
	rightPower = 0;
	isLeftForward = false;
	isRightForward = false;
	leftFront->Set(0);
	leftRear->Set(0);
	rightFront->Set(0);
	rightRear->Set(0);
}

double DriveSystem::GetPowerLeft() {
	return leftPower;
}

double DriveSystem::GetPowerRight() {
	return rightPower;
}

bool DriveSystem::IsLeftForward() {
	return DriveSystem::leftEncoder->GetDirection();
}

bool DriveSystem::IsRightForward() {
	return DriveSystem::rightEncoder->GetDirection();
}

//NavX-MXP actions
void DriveSystem::ResetYaw() {
	DriveSystem::navx->Reset();
}

void DriveSystem::ResetDisplacement() {
	DriveSystem::navx->ResetDisplacement();
}

//NavX-MXP readings
double DriveSystem::GetPitch() {
	return DriveSystem::navx->GetPitch();
}

double DriveSystem::GetYaw() {
	return DriveSystem::navx->GetYaw();
}

double DriveSystem::GetContinuousYaw() {
	return DriveSystem::navx->GetAngle();
}

double DriveSystem::GetRoll() {
	return DriveSystem::navx->GetRoll();
}

double DriveSystem::GetDisplacementX() {
	return DriveSystem::navx->GetDisplacementX();
}

double DriveSystem::GetDisplacementY() {
	return DriveSystem::navx->GetDisplacementY();
}

double DriveSystem::GetDisplacementZ() {
	return DriveSystem::navx->GetDisplacementZ();
}

void DriveSystem::ResetEncoders() {
	DriveSystem::leftEncoder->Reset();
	DriveSystem::rightEncoder->Reset();
}

int DriveSystem::GetRawEncoder(EncoderSide whichSide) {
	if (whichSide == kLeft) {
		return DriveSystem::leftEncoder->GetRaw();
	} else {
		return DriveSystem::rightEncoder->GetRaw();
	}
}

double DriveSystem::GetEncoderDistance(EncoderSide whichSide) {
	if (whichSide == kLeft) {
		return DriveSystem::leftEncoder->GetDistance();
	} else {
		return DriveSystem::rightEncoder->GetDistance();
	}
}

double DriveSystem::GetEncoderRate(EncoderSide whichSide) {
	if (whichSide == kLeft) {
		return DriveSystem::leftEncoder->GetRate();
	} else {
		return DriveSystem::rightEncoder->GetRate();
	}
}
