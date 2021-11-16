// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <units/length.h>

#include <iostream>
#include <wpi/numbers>

#include "Constants.h"

/// Ticks per revolution on the Rev Neo motors.
constexpr double kTicksPerRevolution_NeoMotor = 42;

/// Gear ratio used for the 2020/2021 robots.
constexpr double kGearRatio_2021 = 10.71;

/// Wheel diameter on the 2020/2021 robots.
/// Makes use of the "units" library provided by WPI.  (See docs at
/// https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html.)
static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

/// The tab used for commands/debugging widgets that are specific to this
/// subsystem.
static const char *const kShuffleboardTabName = "DriveBase";

DriveBase::DriveBase()
    :
#ifdef ENABLE_SPARK_MAX_VIA_CAN
      m_leftFront(CANBusIds::SparkMax::Left_Front_No,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_leftRear(CANBusIds::SparkMax::Left_Rear_No,
                 rev::CANSparkMax::MotorType::kBrushless),
      m_rightFront(CANBusIds::SparkMax::Right_Front_No,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_rightRear(CANBusIds::SparkMax::Right_Rear_No,
                  rev::CANSparkMax::MotorType::kBrushless),
#endif // ENABLE_SPARK_MAX_VIA_CAN
      m_odometry(units::degree_t(0))
{
  SetSubsystem("Drivebase");

#ifdef ENABLE_SPARK_MAX_VIA_CAN
  // The following is why we need to dynamically allocate the speed controller
  // groups and differential drive: we have to mark motors as inverted (or not)
  // *before* putting them into the groups (and then those into the drive); if
  // we do it afterward, it just fails (silently).
  m_leftFront.SetInverted(false);
  m_leftRear.SetInverted(false);
  m_rightFront.SetInverted(true);
  m_rightRear.SetInverted(true);

  leftMotors.reset(new frc::SpeedControllerGroup{m_leftFront, m_leftRear});
  rightMotors.reset(new frc::SpeedControllerGroup{m_rightFront, m_rightRear});

  ConfigureEncoders();

  drive.reset(new frc::DifferentialDrive{*leftMotors, *rightMotors});
#endif // ENABLE_SPARK_MAX_VIA_CAN

  SetCoastingEnabled(false);

  // Gyro must be calibrated to initialize for use (generally immediately on
  // start-up).
  m_adiGyro.Calibrate();

  ConfigureShuffleboard();
}

void DriveBase::ConfigureShuffleboard()
{
  auto &tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);

#ifdef ENABLE_SPARK_MAX_VIA_CAN
  leftDistance =
      tab.AddPersistent("L Distance", m_leftRearEncoder.GetPosition())
          .GetEntry();
  rightDistance =
      tab.AddPersistent("R Distance", m_rightRearEncoder.GetPosition())
          .GetEntry();
  leftSpeed =
      tab.AddPersistent("L Speed", m_leftRearEncoder.GetVelocity()).GetEntry();
  rightSpeed =
      tab.AddPersistent("R Speed", m_rightRearEncoder.GetVelocity()).GetEntry();
#endif // ENABLE_SPARK_MAX_VIA_CAN

  rotation =
      tab.AddPersistent("Rotation",
                        GetZAxisGyro().GetRotation2d().Degrees().to<double>())
          .GetEntry();
  tab.Add("Reset Odometry", m_resetCommand);
}

void DriveBase::ConfigureEncoders()
{
  // Default for the encoders is to report velocity in RPM; we want that to come
  // back as m/s.
  units::meter_t wheelCircumference =
      kWheelDiameter * wpi::numbers::pi; // Will auto-convert to meters
  units::meter_t velocityAdjustmentForGearing =
      wheelCircumference / kGearRatio_2021; // Should convert RPM to m/min
  units::meter_t velocityAdjustment =
      velocityAdjustmentForGearing / 60; // Adjust to m/s

  std::cout << "Wheel circumference: " << wheelCircumference.value()
            << wheelCircumference.abbreviation()
            << "\nVelocity adj. (gearing): "
            << velocityAdjustmentForGearing.value()
            << velocityAdjustmentForGearing.abbreviation()
            << "\nVelocity adj. (final): " << velocityAdjustment.value()
            << velocityAdjustment.abbreviation() << std::endl;

#ifdef ENABLE_SPARK_MAX_VIA_CAN
  m_leftFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_leftRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
#endif // ENABLE_SPARK_MAX_VIA_CAN

  ResetEncoders();
}

void DriveBase::UpdateOdometry()
{
  auto currentRotation = m_adiGyro.GetRotation2d();
  auto currentLeftDistance = GetLeftDistance();
  auto currentRightDistance = GetRightDistance();

  m_odometry.Update(currentRotation, currentLeftDistance, currentRightDistance);
}

// This method will be called once per scheduler run
void DriveBase::Periodic()
{
  // Update odometry data (REQUIRED for trajectory following).
  UpdateOdometry();

  // Update displayed data for debugging/monitoring.
  UpdateShuffleboard();
}

void DriveBase::SetCoastingEnabled(bool enabled)
{
#ifdef ENABLE_SPARK_MAX_VIA_CAN
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  m_leftFront.SetIdleMode(mode);
  m_leftRear.SetIdleMode(mode);
  m_rightFront.SetIdleMode(mode);
  m_rightRear.SetIdleMode(mode);
#endif // ENABLE_SPARK_MAX_VIA_CAN
}

void DriveBase::ResetEncoders()
{
#ifdef ENABLE_SPARK_MAX_VIA_CAN
  m_leftFrontEncoder.SetPosition(0.0);
  m_leftRearEncoder.SetPosition(0.0);
  m_rightFrontEncoder.SetPosition(0.0);
  m_rightRearEncoder.SetPosition(0.0);
#endif // ENABLE_SPARK_MAX_VIA_CAN
}

double DriveBase::GetLeftEncoderCount()
{
#ifdef ENABLE_SPARK_MAX_VIA_CAN
  // For our encoders on Mae/Nike, it's the # of revolutions
  // TODO(mjh): Decide if we want to just use 1 encoder per side, or take an
  // average, or what.
  return m_leftFrontEncoder.GetPosition();
#else
  return 0;
#endif // ENABLE_SPARK_MAX_VIA_CAN
}

double DriveBase::GetRightEncoderCount()
{
#ifdef ENABLE_SPARK_MAX_VIA_CAN
  // For our encoders on Mae/Nike, it's the # of revolutions
  return m_rightFrontEncoder.GetPosition();
#else
  return 0;
#endif // ENABLE_SPARK_MAX_VIA_CAN
}

frc::DifferentialDriveWheelSpeeds DriveBase::GetWheelSpeeds()
{
#ifdef ENABLE_SPARK_MAX_VIA_CAN
  return frc::DifferentialDriveWheelSpeeds{
      m_leftRearEncoder.GetVelocity() * 1_m / 1_s,
      m_rightRearEncoder.GetVelocity() * 1_m / 1_s};
#else
  return frc::DifferentialDriveWheelSpeeds{0_mps, 0_mps};
#endif // ENABLE_SPARK_MAX_VIA_CAN
}

units::meter_t DriveBase::GetRightDistance()
{
  auto rightDistance = (GetRightEncoderCount() / kGearRatio_2021) *
                       (kWheelDiameter * wpi::numbers::pi);
  return rightDistance; // Units will automatically convert
}

units::meter_t DriveBase::GetLeftDistance()
{
  auto leftDistance = (GetLeftEncoderCount() / kGearRatio_2021) *
                      (kWheelDiameter * wpi::numbers::pi);
  return leftDistance; // Units will automatically convert
}

frc::Pose2d DriveBase::GetPose()
{
  return m_odometry.GetPose();
}

void DriveBase::ResetOdometry(frc::Pose2d pose)
{
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_adiGyro.GetRotation2d());
}

void DriveBase::TankDriveVolts(units::volt_t left, units::volt_t right)
{
  if (leftMotors)
  {
    leftMotors->SetVoltage(left);
  }
  if (rightMotors)
  {
    rightMotors->SetVoltage(right);
  }
  if (drive)
  {
    drive->Feed();
  }
}

void DriveBase::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                            bool squareInputs)
{
  if (drive)
  {
    drive->ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
  }
}

void DriveBase::CurvatureDrive(double xaxisSpeed, double zaxisRotate,
                               bool isQuickTurn)
{
  if (drive)
  {
    drive->CurvatureDrive(xaxisSpeed, zaxisRotate, isQuickTurn);
  }
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed)
{
  std::cout << "Left: " << leftSpeed << ", right: " << rightSpeed << std::endl;
  if (drive)
  {
    drive->TankDrive(leftSpeed, -rightSpeed);
  }
}

void DriveBase::AddToShuffleboard(std::string_view label, wpi::Sendable *data)
{
  if (data != nullptr)
  {
    auto &tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);
    tab.Add(label, *data);
  }
}

void DriveBase::UpdateShuffleboard()
{
  const auto wheelSpeeds = GetWheelSpeeds();

  leftDistance.SetDouble(GetLeftDistance().to<double>());
  rightDistance.SetDouble(GetRightDistance().to<double>());
  leftSpeed.SetDouble(wheelSpeeds.left.to<double>());
  rightSpeed.SetDouble(wheelSpeeds.right.to<double>());

  rotation.SetDouble(m_odometry.GetPose().Rotation().Degrees().to<double>());
}
