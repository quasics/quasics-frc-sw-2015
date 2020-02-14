/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"

#include "Constants.h"
#include "utils/RangeLimiter.h"
#include "utils/ValueScaler.h"

// Allows us to limit values to the legal range of the joystick (in case someone
// feeds us bad data).
const std::function<double(double)> DriveBase::joystickRangeLimiter =
    RangeLimiter<double>(-1.0, 1.0);

const std::function<double(double)> DriveBase::standardPowerAdjuster =
    ValueScaler<double>(DriveBaseConstants::kStandardPowerScalingFactor);

const std::function<double(double)> DriveBase::turboPowerAdjuster =
    ValueScaler<double>(DriveBaseConstants::kTurboPowerScalingFactor);

DriveBase::DriveBase()
    : leftFront(CANBusConstants::SparkMaxIds::DriveBaseLeftFrontId,
                rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusConstants::SparkMaxIds::DriveBaseLeftRearId,
               rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusConstants::SparkMaxIds::DriveBaseRightFrontId,
                 rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusConstants::SparkMaxIds::DriveBaseRightRearId,
                rev::CANSparkMax::MotorType::kBrushless),
      powerAdjuster(standardPowerAdjuster) {
  SetSubsystem("DriveBase");
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
}

void DriveBase::SetMotorPower(double leftPower, double rightPower) {
  // Adjust the specified motor power values, based on current constraints.
  const double appliedLeftPower =
      powerAdjuster(joystickRangeLimiter(leftPower));
  const double appliedRightPower =
      powerAdjuster(joystickRangeLimiter(rightPower));

  // Apply power to motors.
  leftFront.Set(appliedLeftPower);
  leftRear.Set(appliedLeftPower);
  rightFront.Set(appliedRightPower);
  rightRear.Set(appliedRightPower);
}