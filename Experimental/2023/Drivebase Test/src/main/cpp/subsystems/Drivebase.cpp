#include "subsystems/Drivebase.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

//figure out what encoder returns so you can transfer
Drivebase::Drivebase()
    :
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}} {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_rightMotors.SetInverted(true);

  // Set the distance per pulse for the encoders

  ResetEncoders();
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

void Drivebase::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // Will be using the front encoders
  m_odometry.Update(m_gyro.GetRotation2d(),
                    GetLeftDistance(),
                    GetRightDistance());


  frc::SmartDashboard::PutNumber("GetWheelSpeeds() Left Velocity", double{GetLeftVelocity()});
  frc::SmartDashboard::PutNumber("GetWheelSpeeds() Right Velocity", double{GetRightVelocity()});
  
  //For Pose 2d how do you know which pose is it returning. Is it an X, Y, and a Rotation, Or an Translation and Rotation
  //frc::SmartDashboard::PutData("GetPose() Translation", GetPose().Translation());

                    
}

void Drivebase::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(right);
  m_drive.Feed();
}

void Drivebase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftBackEncoder.SetPosition(0);
  m_rightBackEncoder.SetPosition(0);
}

units::meter_t Drivebase::GetAverageEncoderDistance() {
  return (GetLeftDistance() + GetRightDistance()) / 2.0;
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

void Drivebase::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t Drivebase::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

double Drivebase::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d Drivebase::GetPose() {
  //this seems to be used
  //figure out what an analog gyro returns when get pose is called Should be the same CHECK
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds() {
    //this seems to be used in the 
    //figure out what the rate of an encoder returns DONE
  return {GetLeftVelocity(),GetRightVelocity()};
}

void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),GetLeftDistance(), GetRightDistance(), pose);
}
