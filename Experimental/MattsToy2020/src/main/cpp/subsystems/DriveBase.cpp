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

/// Ticks per revolution on the Rev Neo motors.
constexpr double kTicksPerRevolution_NeoMotor = 42;

/// Gear ratio used for the 2020 robots.
constexpr double kGearRatio_2020 = 10.71;

/// Wheel diameter on the 2020 robots.
constexpr double kWheelDiameter_Inches_2020 = 6;

/// Constant for Pi.  (Not included in math library until C++20.)
constexpr double PI = 4.0 * std::atan(1);

static constexpr EncoderTicksToUnitsConverter ticksToInchesConverter(
    kTicksPerRevolution_NeoMotor, kGearRatio_2020, kWheelDiameter_Inches_2020);

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
                rev::CANSparkMax::MotorType::kBrushless),
      powerAdjuster(standardPowerAdjuster),
      loggingOn("DriveBase noisy", "Logging") {
  SetSubsystem("DriveBase");
  ResetEncoderPosition(Motors::All);
  // frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Logging");
  // debuggingOnEntry = tab.Add("DriveBase noisy", false)
  //                        .WithWidget(frc::BuiltInWidgets::kToggleSwitch)
  //                        .GetEntry();
  // frc::SmartDashboard::GetBoolean("DriveBase noisy", false);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
  ReportEncoderDataToSmartDashboard();
}

/** Enables "turbo" mode, where we increase maximum speed.  (Will take effect
 * the next time thst SetMotorPower() is invoked.) */
void DriveBase::EnableTurboMode() {
  std::cout << "Enabling turbo mode" << std::endl;
  powerAdjuster = turboPowerAdjuster;
}
/** Disables "turbo" mode.  (Will take effect the next time thst
 * SetMotorPower() is invoked.) */
void DriveBase::DisableTurboMode() {
  std::cout << "Disabling turbo mode" << std::endl;
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

void DriveBase::ReportEncoderDataToSmartDashboard(std::string prefix,
                                                  rev::CANEncoder& encoder) {
  const double posInTicks = encoder.GetPosition();
  const double posInInches = ticksToInchesConverter(posInTicks);

  frc::SmartDashboard::PutNumber(prefix + " pos (tx)", posInTicks);
  frc::SmartDashboard::PutNumber(prefix + " pos (in)", posInInches);
}

void DriveBase::ReportEncoderDataToSmartDashboard() {
  ReportEncoderDataToSmartDashboard("R. front", rightFrontEncoder);
  ReportEncoderDataToSmartDashboard("R. rear", rightRearEncoder);
  ReportEncoderDataToSmartDashboard("L. front", leftFrontEncoder);
  ReportEncoderDataToSmartDashboard("L. rear", leftRearEncoder);
}