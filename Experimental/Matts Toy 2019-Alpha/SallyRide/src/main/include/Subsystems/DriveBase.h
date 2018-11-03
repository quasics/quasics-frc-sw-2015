/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <WPILib.h>
#include "RobotMap.h"

class DriveBase : public frc::Subsystem {
 private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	// TODO: Move port numbers (and other attributes) into RobotMap.h.
	frc::Encoder leftEncoder{8, 9, true};
	frc::Encoder rightEncoder{0, 1, false};
	frc::PWMVictorSPX rightFrontMotor{0};
	frc::PWMVictorSPX rightRearMotor{1};
	frc::PWMVictorSPX leftFrontMotor{2};
	frc::PWMVictorSPX leftRearMotor{3};
	frc::SpeedControllerGroup rightMotors{rightFrontMotor, rightRearMotor};
	frc::SpeedControllerGroup leftMotors{leftFrontMotor, leftRearMotor};

 public:
 	DriveBase();
	void InitDefaultCommand() override;

	void SetPowerToMotors(double leftPercent, double rightPercent);

	double RightEncoderVelocity();
	double LeftEncoderVelocity();
	double RightEncoderDistance();
	double LeftEncoderDistance();
	uint32_t LeftEncoderRaw();
	uint32_t RightEncoderRaw();
	void RightEncoderReset();
	void LeftEncoderReset();

	void Stop();
};
