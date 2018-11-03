/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/DriveBase.h"
#include "RobotMap.h"
#include "EncoderVariables.h"

template <class Widget>
inline void setNameAndSubsystem(Widget &w, const char *const subsystem, const char *const name)
{
	w.SetSubsystem(subsystem);
	w.SetName(name);
}
inline void configureEncoder(
	Encoder &encoder, double distancePerPulse,
	frc::PIDSourceType sourceType,
	const char *const subsystem, const char *const name)
{
	encoder.SetDistancePerPulse(distancePerPulse);
	encoder.SetPIDSourceType(sourceType);
	setNameAndSubsystem(encoder, subsystem, name);
}

DriveBase::DriveBase() : Subsystem("DriveBase")
{
	setNameAndSubsystem(leftFrontMotor, "DriveBase", "leftFront");
	setNameAndSubsystem(leftRearMotor, "DriveBase", "leftRear");
	setNameAndSubsystem(rightFrontMotor, "DriveBase", "rightFront");
	setNameAndSubsystem(rightRearMotor, "DriveBase", "rightRear");

	setNameAndSubsystem(leftMotors, "DriveBase", "Left Motors");
	setNameAndSubsystem(rightMotors, "DriveBase", "Right Motors");

	configureEncoder(leftEncoder, DRIVE_TRAIN_INCHES_PER_TICK,
					 frc::PIDSourceType::kRate, "DriveBase", "Left Encoder");
	configureEncoder(rightEncoder, DRIVE_TRAIN_INCHES_PER_TICK,
					 frc::PIDSourceType::kRate, "DriveBase", "Right Encoder");
}

void DriveBase::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

///////////////////////////////////////////////////////////////////////////
// Motor control starts here

void DriveBase::SetPowerToMotors(double leftPercent, double rightPercent)
{
	leftMotors.Set(leftPercent);
	rightMotors.Set(rightPercent);
}

void DriveBase::Stop()
{
	SetPowerToMotors(0, 0);
}

// Motor control ends here
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// Encoders start here

double DriveBase::RightEncoderVelocity()
{
	return rightEncoder.GetRate();
}

double DriveBase::LeftEncoderVelocity()
{
	return leftEncoder.GetRate();
}

double DriveBase::RightEncoderDistance()
{
	return rightEncoder.GetDistance();
}
double DriveBase::LeftEncoderDistance()
{
	return leftEncoder.GetDistance();
}
uint32_t DriveBase::LeftEncoderRaw()
{
	return rightEncoder.GetRaw();
}
uint32_t DriveBase::RightEncoderRaw()
{
	return leftEncoder.GetRaw();
}

void DriveBase::RightEncoderReset()
{
	leftEncoder.Reset();
}
void DriveBase::LeftEncoderReset()
{
	rightEncoder.Reset();
}

//Encoders end here
///////////////////////////////////////////////////////////////////////////
