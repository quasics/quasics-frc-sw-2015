/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Drivebase.h"

#include "utils/EncoderHelpers.h"
#include "utils/ShuffleboardWrappers.h"

Drivebase::Drivebase()
    : leftFront(CANBusIds::SparkMax::Left_Front_No,
                rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusIds::SparkMax::Left_Rear_No,
               rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusIds::SparkMax::Right_Front_No,
                 rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusIds::SparkMax::Right_Rear_No,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");
  leftFront.SetInverted(true);
  leftRear.SetInverted(true);
  SetCoastingEnabled(false);
}
// This method will be called once per scheduler run
void Drivebase::Periodic() {
  DisplayEncoderValues();
}

void Drivebase::DisplayEncoderValues() {
  static const std::string kEncoderTabName = this->GetSubsystem();
  static ShuffleboardWrappers::SimpleDisplay leftFrontPosition(
      "Left Front Encoder Position", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay leftFrontVelocity(
      "Left Front Encoder Velocity", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay leftRearPosition(
      "Left Rear Encoder Position", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay leftRearVelocity(
      "Left Rear Encoder Velocity", kEncoderTabName);

  static ShuffleboardWrappers::SimpleDisplay rightFrontPosition(
      "Right Front Encoder Position", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay rightFrontVelocity(
      "Right Front Encoder Velocity", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay rightRearPosition(
      "Right Rear Encoder Position", kEncoderTabName);
  static ShuffleboardWrappers::SimpleDisplay rightRearVelocity(
      "Right Rear Encoder Velocity", kEncoderTabName);

  leftFrontPosition.SetValue(leftFrontEncoder.GetPosition());
  leftFrontVelocity.SetValue(leftFrontEncoder.GetVelocity());
  leftRearPosition.SetValue(leftRearEncoder.GetPosition());
  leftRearVelocity.SetValue(leftRearEncoder.GetVelocity());

  rightFrontPosition.SetValue(rightFrontEncoder.GetPosition());
  rightFrontVelocity.SetValue(rightFrontEncoder.GetVelocity());
  rightRearPosition.SetValue(rightRearEncoder.GetPosition());
  rightRearVelocity.SetValue(rightRearEncoder.GetVelocity());
}

double Drivebase::GetLeftFrontEncoderPosition() {
  // Note: we're negating the values to match live behavior, given the motor's
  // orientation (e.g., so that we get positive numbers when the motor is moving
  // us forward).
  return -leftFrontEncoder.GetPosition();
}

double Drivebase::GetRightFrontEncoderPosition() {
  // Note: we're negating the values to match live behavior, given the motor's
  // orientation (e.g., so that we get positive numbers when the motor is moving
  // us forward).
  return -rightFrontEncoder.GetPosition();
}

void Drivebase::ResetEncoderPositions() {
  leftFrontEncoder.SetPosition(0.0);
  leftRearEncoder.SetPosition(0.0);
  rightFrontEncoder.SetPosition(0.0);
  rightRearEncoder.SetPosition(0.0);
}

void Drivebase::SetMotorPower(double rightPower, double leftPower) {
  if (frontIsForward) {
    leftFront.Set(leftPower * powerScaling);
    leftRear.Set(leftPower * powerScaling);
    rightFront.Set(rightPower * powerScaling);
    rightRear.Set(rightPower * powerScaling);
  } else {
    leftFront.Set(-rightPower * powerScaling);
    leftRear.Set(-rightPower * powerScaling);
    rightFront.Set(-leftPower * powerScaling);
    rightRear.Set(-leftPower * powerScaling);
  }
}

void Drivebase::SetCoastingEnabled(bool enabled) {
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  leftFront.SetIdleMode(mode);
  leftRear.SetIdleMode(mode);
  rightFront.SetIdleMode(mode);
  rightRear.SetIdleMode(mode);
}

constexpr double kTicksPerRevolution_NeoMotor = 42;
constexpr double kGearRatio_2020 = 10.71;
constexpr double kWheelDiameter_Inches_2020 = 6;

static constexpr EncoderRevolutionsToUnitsConverter
    EncoderRevolutionsToUnitsConverter(kGearRatio_2020,
                                       kWheelDiameter_Inches_2020);

double Drivebase::GetLeftEncoderInInches() {
  return EncoderRevolutionsToUnitsConverter(leftFrontEncoder.GetPosition());
}
double Drivebase::GetRightEncoderInInches() {
  return EncoderRevolutionsToUnitsConverter(rightFrontEncoder.GetPosition());
}
