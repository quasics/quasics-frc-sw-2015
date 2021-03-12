// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>

#include <wpi/math>

#include "Constants.h"

/// Ticks per revolution on the Rev Neo motors.
constexpr double kTicksPerRevolution_NeoMotor = 42;

/// Gear ratio used for the 2020/2021 robots.
constexpr double kGearRatio_2021 = 10.71;

/// Wheel diameter on the 2020/2021 robots.
/// Makes use of the "units" library provided by WPI.  (See docs at
/// https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html.)
static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

static const char* const kShuffleboardTabName = "DriveBase";

DriveBase::DriveBase()
    : m_leftFront(CANBusIds::SparkMax::Left_Front_No,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_leftRear(CANBusIds::SparkMax::Left_Rear_No,
                 rev::CANSparkMax::MotorType::kBrushless),
      m_rightFront(CANBusIds::SparkMax::Right_Front_No,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_rightRear(CANBusIds::SparkMax::Right_Rear_No,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_odometry(units::degree_t(0)) {
  SetSubsystem("Drivebase");

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

  SetCoastingEnabled(false);

  // Gyro must be calibrated to initialize for use (generally immediately on
  // start-up).
  m_adiGyro.Calibrate();

  ConfigureShuffleboard();
}

void DriveBase::ConfigureShuffleboard() {
  auto& tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);
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
  rotation =
      tab.AddPersistent("Rotation",
                        GetZAxisGyro().GetRotation2d().Degrees().to<double>())
          .GetEntry();
  tab.Add("Reset Odometry", m_resetCommand);
}

void DriveBase::AddToShuffleboard(wpi::StringRef label, frc::Sendable* data,
                                  bool isPersistent) {
  auto& tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);
  if (isPersistent) {
    tab.AddPersistent(label, data);
  } else {
    tab.Add(label, data);
  }
}

void DriveBase::UpdateShuffleboard() {
  const auto wheelSpeeds = GetWheelSpeeds();

  leftDistance.SetDouble(GetLeftDistance().to<double>());
  rightDistance.SetDouble(GetRightDistance().to<double>());
  leftSpeed.SetDouble(wheelSpeeds.left.to<double>());
  rightSpeed.SetDouble(wheelSpeeds.right.to<double>());
  rotation.SetDouble(m_odometry.GetPose().Rotation().Degrees().to<double>());
}

void DriveBase::ConfigureEncoders() {
  // Default for the encoders is to report velocity in RPM; we want that to come
  // back as m/s.
  units::meter_t wheelCircumference =
      kWheelDiameter * wpi::math::pi;  // Will auto-convert to meters
  units::meter_t velocityAdjustmentForGearing =
      wheelCircumference / kGearRatio_2021;  // Should convert RPM to m/min
  units::meter_t velocityAdjustment =
      velocityAdjustmentForGearing / 60;  // Adjust to m/s

  std::cout << "Wheel circumference: " << wheelCircumference
            << "\nVelocity adj. (gearing): " << velocityAdjustmentForGearing
            << "\nVelocity adj. (final): " << velocityAdjustment << std::endl;

  m_leftFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_leftRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());

  ResetEncoders();
}

void DriveBase::UpdateOdometry() {
  auto currentRotation = m_adiGyro.GetRotation2d();
  auto currentLeftDistance = GetLeftDistance();
  auto currentRightDistance = GetRightDistance();

  m_odometry.Update(currentRotation, currentLeftDistance, currentRightDistance);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
  // Update odometry data (REQUIRED for trajectory following).
  UpdateOdometry();

  // Update displayed data for debugging/monitoring.
  UpdateShuffleboard();
}

void DriveBase::SetCoastingEnabled(bool enabled) {
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  m_leftFront.SetIdleMode(mode);
  m_leftRear.SetIdleMode(mode);
  m_rightFront.SetIdleMode(mode);
  m_rightRear.SetIdleMode(mode);
}

void DriveBase::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                            bool squareInputs) {
  drive->ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  std::cout << "Left: " << leftSpeed << ", right: " << rightSpeed << std::endl;
  drive->TankDrive(leftSpeed, -rightSpeed);
}

void DriveBase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0.0);
  m_leftRearEncoder.SetPosition(0.0);
  m_rightFrontEncoder.SetPosition(0.0);
  m_rightRearEncoder.SetPosition(0.0);
}

double DriveBase::GetLeftEncoderCount() {
  // For our encoders on Mae/Nike, it's the # of revolutions
  // TODO(mjh): Decide if we want to just use 1 encoder per side, or take an
  // average, or what.
  return m_leftFrontEncoder.GetPosition();
}

double DriveBase::GetRightEncoderCount() {
  // For our encoders on Mae/Nike, it's the # of revolutions
  return m_rightFrontEncoder.GetPosition();
}

units::meter_t DriveBase::GetRightDistance() {
  auto rightDistance = ((m_rightFrontEncoder.GetPosition()) / kGearRatio_2021) *
                       (kWheelDiameter * wpi::math::pi);
  return rightDistance;  // Units will automatically convert
}

units::meter_t DriveBase::GetLeftDistance() {
  auto leftDistance = ((m_leftFrontEncoder.GetPosition()) / kGearRatio_2021) *
                      (kWheelDiameter * wpi::math::pi);
  return leftDistance;  // Units will automatically convert
}

frc::Pose2d DriveBase::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveBase::GetWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{
      m_leftRearEncoder.GetVelocity() * 1_m / 1_s,
      m_rightRearEncoder.GetVelocity() * 1_m / 1_s};
}

void DriveBase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_adiGyro.GetRotation2d());
}

void DriveBase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  leftMotors->SetVoltage(left);
  rightMotors->SetVoltage(right);
  drive->Feed();
}
