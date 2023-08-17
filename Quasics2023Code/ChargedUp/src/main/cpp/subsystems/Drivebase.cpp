// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>

#include <iostream>

#if defined(ENABLE_AD_GYRO) && defined(ENABLE_PIGEON)
static_assert(false, "Csan't enable both AD and Pigeon gyros!");
#endif

Drivebase::Drivebase() {
  SetName("Drivebase");

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  m_leftFront.SetInverted(false);
  m_leftBack.SetInverted(false);
  m_rightFront.SetInverted(true);
  m_rightBack.SetInverted(true);

  ConfigureEncoders();

  // Shouldn't be required for Pigeon, but would be for other gyros.  (So,
  // better safe than sorry....)
#ifdef ENABLE_PIGEON
  m_pigeon.Calibrate();
  m_pigeon.Reset();
  m_pitchShift = m_pigeon.GetPitch();
#elif defined(ENABLE_AD_GYRO)
  m_adGyro.Calibrate();
#endif

  // Establish an initial, known braking mode
  SetBrakingMode(false);
}

void Drivebase::SetBrakingMode(bool enabled) {
  m_isBraking = enabled;
  if (enabled) {
    m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else {
    m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }
}

void Drivebase::ConfigureEncoders() {
  // Calculate wheel circumference (distance travelled per wheel revolution).
  const double pi = 3.1415926;
  const units::meter_t wheelCircumference = RobotPhysics::WHEEL_DIAMETER * pi;

  // Compute distance traveled per rotation of the motor.
  const units::meter_t gearingConversion =
      wheelCircumference / RobotPhysics::DRIVEBASE_GEAR_RATIO;

  // Compute conversion factor (used to change "(motor) RPM" to "m/sec").
  const units::meter_t velocityCorrection = gearingConversion / 60;

  // Update encoders so that they will report distance as meters traveled,
  // rather than rotations.
  m_leftFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());

  // Update encoders so that they will report velocity as m/sec, rather than
  // RPM.
  m_leftFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

  ResetEncoders();
}

units::meter_t Drivebase::GetLeftDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_leftFrontEncoder.GetPosition());
}

units::meter_t Drivebase::GetRightDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_rightFrontEncoder.GetPosition());
}

units::meters_per_second_t Drivebase::GetLeftVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_leftFrontEncoder.GetVelocity());
}

units::meters_per_second_t Drivebase::GetRightVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_rightFrontEncoder.GetVelocity());
}

void Drivebase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftBackEncoder.SetPosition(0);
  m_rightBackEncoder.SetPosition(0);
}

units::degree_t Drivebase::GetYaw() {
#if defined(ENABLE_PIGEON)
  return m_pigeon.GetRotation2d().Degrees();
#else
  return m_adGyro.GetAngle() *
         -1_deg;  // convert double returned by GetAngle() to degrees. multiply
                  // by -1 because values are flipped for some reason
#endif
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{GetLeftVelocity(),
                                           GetRightVelocity()};
}

void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(m_pigeon.GetRotation2d(), 0_m, 0_m, pose);
}

frc::Pose2d Drivebase::GetPose() {
  return m_odometry.GetPose();
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {
  frc::SmartDashboard::PutString(
      "DriveBase Mode", IsInBrakingMode() ? "Brake Mode" : "Coast Mode");
  /*
  frc::SmartDashboard::PutNumber("Pitch", GetPitchImpl());
  frc::SmartDashboard::PutNumber("Roll", GetRollImpl());
  */
  // frc::SmartDashboard::PutNumber("Yaw:", GetYaw().value());
  //  std::cerr << "Yaw: " << GetAngle().value() << std::endl;

  auto angle = GetYaw();
  auto leftDistance = GetLeftDistance();
  auto rightDistance = GetRightDistance();
  m_odometry.Update(angle, leftDistance, rightDistance);
}

void Drivebase::TankDrive(double leftPower, double rightPower) {
  m_drive->TankDrive(leftPower, rightPower);
}

void Drivebase::ArcadeDrive(double power, double angle) {
  m_drive->ArcadeDrive(power, angle);
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftSide->SetVoltage(left);
  m_rightSide->SetVoltage(right);
  m_drive->Feed();
}

double Drivebase::GetPitchImpl() {
#ifdef ENABLE_PIGEON
  return m_pigeon.GetPitch() - m_pitchShift;
#else  // ad gyro cannot get pitch
  return 0;
#endif
}

double Drivebase::GetRollImpl() {
#ifdef ENABLE_PIGEON
  return m_pigeon.GetRoll() - m_rollShift;
#else  // ad gyro cannot get pitch
  return 0;
#endif
}

void Drivebase::GyroCalibration() {
#ifdef ENABLE_PIGEON
  m_pitchShift = m_pigeon.GetPitch();
  m_rollShift = m_pigeon.GetRoll();
#elif defined ENABLE_AD_GYRO
  m_adGyro.Calibrate();
  m_pitchShift = 0;
  m_rollShift = 0;
#endif
}
