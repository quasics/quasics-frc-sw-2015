/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Quasics Robotics and Matthew J. Healy                   */
/* All Rights Reserved.                                                       */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the BSD license file in the root directory of the   */
/* project repository.                                                        */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

#include "Constants.h"
#include "utils/EncoderHelpers.h"
#include "utils/RangeLimiter.h"
#include "utils/ValueScaler.h"

/// List of DriveBase motor specifiers that map to a single motor (vs. a group).
static constexpr auto kPrimaryMotorPositions = {
    DriveBase::Motors::LeftFront, DriveBase::Motors::LeftRear,
    DriveBase::Motors::RightFront, DriveBase::Motors::RightRear};

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
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("DriveBase");
  ResetEncoderPosition(Motors::All);
  SetCoastingEnabled(false);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
  ReportEncoderDataToShuffleboard();
}

/** Enables "turbo" mode, where we increase maximum speed.  (Will take effect
 * the next time thst SetMotorPower() is invoked.) */
void DriveBase::EnableTurboMode() {
  if (loggingOn.GetValue()) {
    std::cout << "Enabling turbo mode" << std::endl;
  }
  powerAdjuster = turboPowerAdjuster;
}
/** Disables "turbo" mode.  (Will take effect the next time thst
 * SetMotorPower() is invoked.) */
void DriveBase::DisableTurboMode() {
  if (loggingOn.GetValue()) {
    std::cout << "Disabling turbo mode" << std::endl;
  }
  powerAdjuster = standardPowerAdjuster;
}

void DriveBase::SetMotorPower(double leftPower, double rightPower) {
  // Adjust the specified motor power values, based on current constraints.
  const double appliedLeftPower =
      powerAdjuster(joystickRangeLimiter(leftPower));
  const double appliedRightPower =
      powerAdjuster(joystickRangeLimiter(rightPower));

  if (loggingOn.GetValue()) {
    std::cout << "Drive power: leftRaw=" << leftPower
              << ", rightRaw=" << rightPower
              << " | leftAdj=" << appliedLeftPower
              << ", rightAdj=" << appliedRightPower << std::endl;
  }

  // Apply power to motors.
  leftFront.Set(appliedLeftPower);
  leftRear.Set(appliedLeftPower);
  rightFront.Set(appliedRightPower);
  rightRear.Set(appliedRightPower);
}

void DriveBase::SetCoastingEnabled(bool enabled) {
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  leftFront.SetIdleMode(mode);
  leftRear.SetIdleMode(mode);
  rightFront.SetIdleMode(mode);
  rightRear.SetIdleMode(mode);
}

rev::CANEncoder* DriveBase::GetEncoder(DriveBase::Motors motor) {
  // Map the enumerated value to a (single) motor encoder, if possible.
  switch (motor) {
    case Motors::LeftFront:
      return &leftFrontEncoder;
    case Motors::LeftRear:
      return &leftRearEncoder;
    case Motors::RightFront:
      return &rightFrontEncoder;
    case Motors::RightRear:
      return &rightRearEncoder;
    default:
      return nullptr;
  }
}

double DriveBase::GetEncoderPosition(DriveBase::Motors motor) {
  int count = 0;
  double positionTotal = 0.0;

  for (Motors m : kPrimaryMotorPositions) {
    if (int(m) & int(motor)) {
      auto* encoder = GetEncoder(m);
      if (encoder) {
        ++count;
        positionTotal += encoder->GetPosition();
      }
    }
  }

  double result = count > 1 ? (positionTotal / count) : positionTotal;
  return result;
}

void DriveBase::ResetEncoderPosition(DriveBase::Motors motor) {
  for (Motors m : kPrimaryMotorPositions) {
    if (int(m) & int(motor)) {
      auto* encoder = GetEncoder(m);
      if (encoder) {
        encoder->SetPosition(0.0);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Debugging support: reporting encoder data to the Smart Dashboard/Shuffleboad

/// Ticks per revolution on the Rev Neo motors.
constexpr double kTicksPerRevolution_NeoMotor = 42;

/// Gear ratio used for the 2020 robots.
constexpr double kGearRatio_2020 = 10.71;

/// Wheel diameter on the 2020 robots.
constexpr double kWheelDiameter_Inches_2020 = 6;

/// Converts "ticks" from the encoders to inches.
static constexpr EncoderRevolutionsToUnitsConverter revsToInchesConverter(
    kGearRatio_2020, kWheelDiameter_Inches_2020);

void DriveBase::ReportEncoderDataToShuffleboard() {
  leftFrontEncoderTicksDisplay.SetValue(leftFrontEncoder.GetPosition());
  leftRearEncoderTicksDisplay.SetValue(leftRearEncoder.GetPosition());
  rightFrontEncoderTicksDisplay.SetValue(rightFrontEncoder.GetPosition());
  rightRearEncoderTicksDisplay.SetValue(rightRearEncoder.GetPosition());

  leftFrontEncoderInchesDisplay.SetValue(
      revsToInchesConverter(leftFrontEncoder.GetPosition()));
  leftRearEncoderInchesDisplay.SetValue(
      revsToInchesConverter(leftRearEncoder.GetPosition()));
  rightFrontEncoderInchesDisplay.SetValue(
      revsToInchesConverter(rightFrontEncoder.GetPosition()));
  rightRearEncoderInchesDisplay.SetValue(
      revsToInchesConverter(rightRearEncoder.GetPosition()));
}
