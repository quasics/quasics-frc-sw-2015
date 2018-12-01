/*
 * DriveBase.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: healym
 */

#include "DriveBase.h"
#include "../RobotMap.h"

#include <SpeedControllerGroup.h>
#include <Commands/Subsystem.h>
#include <Drive/DifferentialDrive.h>

DriveBase::DriveBase()
	: Subsystem("DriveBase")
{
    leftFrontMotor = RobotMap::driveBaseLeftFrontMotor;
    leftRearMotor = RobotMap::driveBaseLeftRearMotor;
    leftMotors = RobotMap::driveBaseLeftMotors;
    rightFrontMotor = RobotMap::driveBaseRightFrontMotor;
    rightRearMotor = RobotMap::driveBaseRightRearMotor;
    rightMotors = RobotMap::driveBaseRightMotors;
}

void DriveBase::InitDefaultCommand() {
    // Set the default command (if any) for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

///////////////////////////////////////////////////////////////////////////
// Motor control starts here

void DriveBase::SetPowerToMotors(double leftPercent, double rightPercent) {
	leftMotors->Set(leftPercent);
	rightMotors->Set(rightPercent);
}

void DriveBase::Stop(){
	SetPowerToMotors(0, 0);
}

// Motor control ends here
///////////////////////////////////////////////////////////////////////////
