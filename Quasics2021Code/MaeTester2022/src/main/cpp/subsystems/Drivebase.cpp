// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/time.h>

#include <iostream>

#include "Constants.h"

#define USE_SPARKS_VIA_CAN

#define EXECUTE_EVERY_N_TIMES(n, outputCmd)   \
  {                                           \
    static int counter = -1;                  \
    if ((counter = (counter + 1) % n) == 0) { \
      outputCmd;                              \
    }                                         \
  }

static constexpr double kTicksPerRevolution_NeoMotor = 42;

static constexpr double kGearRatio_2021 = 10.71;

static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

/// Encapsulates all of the stuff used to communicate with the
/// Spark Max motors over CAN, so that it will be easier to port the project
/// over to the Beta 1 release.
///
/// RevRobotics doesn't have updated libraries for 2022 ready yet, which
/// means that this stuff will all need to be disabled, and it's easier when all
/// of the references are in a single file.
struct Drivebase::RevRoboticsStuff {
#ifdef USE_SPARKS_VIA_CAN
  rev::CANSparkMax m_leftFront{CANBusIds::SparkMaxIds::Left_Front_Number,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftRear{CANBusIds::SparkMaxIds::Left_Rear_Number,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{CANBusIds::SparkMaxIds::Right_Front_Number,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightRear{CANBusIds::SparkMaxIds::Right_Rear_Number,
                               rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::CANEncoder m_leftRearEncoder = m_leftRear.GetEncoder();
  rev::CANEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::CANEncoder m_rightRearEncoder = m_rightRear.GetEncoder();
#endif  // USE_SPARKS_VIA_CAN

  void ResetEncoders() {
#ifdef USE_SPARKS_VIA_CAN
    m_leftFrontEncoder.SetPosition(0.0);
    m_leftRearEncoder.SetPosition(0.0);
    m_rightRearEncoder.SetPosition(0.0);
    m_rightFrontEncoder.SetPosition(0.0);
#endif  // USE_SPARKS_VIA_CAN
  }

  double GetLeftEncoderCount() {
#ifdef USE_SPARKS_VIA_CAN
    return m_leftFrontEncoder.GetPosition();
#else
    return 0;
#endif  // USE_SPARKS_VIA_CAN
  }

  double GetRightEncoderCount() {
#ifdef USE_SPARKS_VIA_CAN
    return m_rightFrontEncoder.GetPosition();
#else
    return 0;
#endif  // USE_SPARKS_VIA_CAN
  }

  double GetLeftVelocity() {
#ifdef USE_SPARKS_VIA_CAN
    return m_leftFrontEncoder.GetVelocity();
#else
    return 0;
#endif  // USE_SPARKS_VIA_CAN
  }

  double GetRightVelocity() {
#ifdef USE_SPARKS_VIA_CAN
    return m_rightFrontEncoder.GetVelocity();
#else
    return 0;
#endif  // USE_SPARKS_VIA_CAN
  }
};

Drivebase::Drivebase()
    : m_revStuff(new RevRoboticsStuff),
      m_odometry(units::degree_t(m_adiGyro.GetAngle())) {
  SetSubsystem("Drivebase");

#ifdef USE_SPARKS_VIA_CAN
  m_revStuff->m_rightFront.SetInverted(true);
  m_revStuff->m_rightRear.SetInverted(true);

  m_revStuff->m_leftFront.SetInverted(false);
  m_revStuff->m_leftRear.SetInverted(false);

  m_leftMotors.reset(new frc::SpeedControllerGroup(m_revStuff->m_leftFront,
                                                   m_revStuff->m_leftRear));
  m_rightMotors.reset(new frc::SpeedControllerGroup(m_revStuff->m_rightFront,
                                                    m_revStuff->m_rightRear));
#endif  // USE_SPARKS_VIA_CAN

  // Default for the encoders is to report velocity in RPM; we want that to come
  // back as m/s.
  units::meter_t wheelCircumference =
      kWheelDiameter * wpi::numbers::pi;  // Will auto-convert to meters
  units::meter_t velocityAdjustmentForGearing =
      wheelCircumference / kGearRatio_2021;  // Should convert RPM to m/min
  units::meter_t velocityAdjustment =
      velocityAdjustmentForGearing / 60;  // Adjust to m/s
  std::cout << "Wheel circumference: " << wheelCircumference << "\n"
            << "Velocity adjustment: " << velocityAdjustment << std::endl;

#ifdef USE_SPARKS_VIA_CAN
  m_revStuff->m_leftFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_revStuff->m_leftRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_revStuff->m_rightFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_revStuff->m_rightRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());

  m_drive.reset(new frc::DifferentialDrive(*m_leftMotors, *m_rightMotors));
#endif  // USE_SPARKS_VIA_CAN

  ResetEncoders();
  m_adiGyro.Calibrate();
}

Drivebase::~Drivebase() {
  delete m_revStuff;
}

void Drivebase::ResetEncoders() {
  m_revStuff->ResetEncoders();
}

double Drivebase::GetLeftEncoderCount() {
  return m_revStuff->GetLeftEncoderCount();
}

double Drivebase::GetRightEncoderCount() {
  return m_revStuff->GetRightEncoderCount();
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{
      m_revStuff->GetLeftVelocity() * 1_m / 1_s,
      m_revStuff->GetRightVelocity() * 1_m / 1_s};
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {
  auto rotation = GetZAxisGyro().GetRotation2d();
  auto leftDistance = GetLeftEncoderDistance();
  auto rightDistance = GetRightEncoderDistance();

  frc::SmartDashboard::PutNumber("L Encoders ", leftDistance.to<double>());
  frc::SmartDashboard::PutNumber("R Encoders", rightDistance.to<double>());
  frc::SmartDashboard::PutNumber("Rotation", rotation.Degrees().to<double>());

  m_odometry.Update(rotation, leftDistance, rightDistance);

  //   EXECUTE_EVERY_N_TIMES(100, auto speeds = GetWheelSpeeds();
  //                         std::cout << "Speeds: " << speeds.left << " / "
  //                                   << speeds.right << std::endl;)
}

void Drivebase::SetMotorSpeed(double leftSpeed, double rightSpeed) {
  //   EXECUTE_EVERY_N_TIMES(50,
  //                         std::cerr << "Setting speeds: left=" << leftSpeed
  //                                   << ", right=" << rightSpeed <<
  //                                   std::endl;)
  if (m_drive) {
    m_drive->TankDrive(leftSpeed, -rightSpeed);
  }
}

void Drivebase::Stop() {
  SetMotorSpeed(0, 0);
}

frc::Pose2d Drivebase::GetPose() {
  return m_odometry.GetPose();
}

void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_adiGyro.GetRotation2d());
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  if (m_leftMotors) {
    m_leftMotors->SetVoltage(left);
  }
  if (m_leftMotors) {
    m_rightMotors->SetVoltage(right);
  }
  if (m_drive) {
    m_drive->Feed();
  }
}

units::meter_t Drivebase::GetRightEncoderDistance() {
  auto distance = (GetRightEncoderCount() / kGearRatio_2021) *
                  (kWheelDiameter * wpi::numbers::pi);
  return distance;
}

units::meter_t Drivebase::GetLeftEncoderDistance() {
  auto distance = (GetLeftEncoderCount() / kGearRatio_2021) *
                  (kWheelDiameter * wpi::numbers::pi);
  return distance;
}
