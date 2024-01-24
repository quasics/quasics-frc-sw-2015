
#include "subsystems/Drivebase.h"
#include <iostream>


#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

//figure out what encoder returns so you can transfer
Drivebase::Drivebase(){
  SetName("Drivebase");
  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  m_leftFront.SetInverted(false);
  m_leftBack.SetInverted(false);
  m_rightFront.SetInverted(true);
  m_rightBack.SetInverted(true);

  ConfigureEncoders();

  //m_gyro.Calibrate();
  m_gyro.Reset();

  //I DONT KNOW IF THIS IS SAFE
  //m_drive->SetSafetyEnabled(false);
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
  //return m_pigeon.GetRotation2d().Degrees();

  // Implementation of subsystem periodic method goes here.
  // Will be using the front encoders

  /*Making it match
  m_odometry.Update(m_gyro.GetRotation2d(),
                    GetLeftDistance(),
                    GetRightDistance());*/
  
  //test
  /*m_odometry.Update(0_deg,
                    GetLeftDistance(),
                    GetRightDistance());*/
                    
  m_odometry.Update(m_gyro.GetRotation2d().Degrees(),
                    GetLeftDistance(),
                    GetRightDistance());
/*
  frc::SmartDashboard::PutNumber("GetWheelSpeeds() Left Velocity", double{GetLeftVelocity()});
  frc::SmartDashboard::PutNumber("GetWheelSpeeds() Right Velocity", double{GetRightVelocity()});
  */
  //For Pose 2d how do you know which pose is it returning. Is it an X, Y, and a Rotation, Or an Translation and Rotation
  //frc::SmartDashboard::PutData("GetPose() Translation", GetPose().Translation());
  frc::SmartDashboard::PutNumber("GetPose.X() Translation in Meters", double{GetPose().X()});
  frc::SmartDashboard::PutNumber("GetPose.Y() Translation in Meters", double{GetPose().Y()});
  //frc::SmartDashboard::PutNumber("GetHeading() Yaw", double(GetHeading()));
  //frc::SmartDashboard::PutNumber("Pitch", double{(m_gyro.GetPitch())});
  //frc::SmartDashboard::PutNumber("Roll", double{(m_gyro.GetRoll())});
  frc::SmartDashboard::PutNumber("IMPORTANT WHAT GYRO IS SENDING", double(m_odometry.GetPose().Rotation().Degrees()));
/*
  frc::SmartDashboard::PutNumber("GetPose.Translation() in Meters", double{GetPose().Translation()});
  frc::SmartDashboard::PutNumber("GetPose.Rotation() in Meters", double{GetPose().Rotation()});*/

  frc::SmartDashboard::PutNumber("Left Wheel Distance", double(GetLeftDistance()));
  frc::SmartDashboard::PutNumber("Right Wheel Distance", double(GetRightDistance()));

                    
}

/*void Drivebase::ArcadeDrive(double fwd, double rot) {
  //unnecessary
  m_drive.ArcadeDrive(fwd, rot);
}*/

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  /*This is a difference TRYING TO REPLICATE

  These lines are used in the constructor of Drivebase.cpp

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  Then these are created in Drivebase.

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;



*/
  m_leftSide->SetVoltage(left);
  m_rightSide->SetVoltage(right);
  m_drive->Feed();
}

void Drivebase::ResetEncoders() {
  std::cout << "Reseting Encoders to 0" << std::endl;
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftBackEncoder.SetPosition(0);
  m_rightBackEncoder.SetPosition(0);
  std::cout << "Left Front Encoder: " << GetLeftDistance().value() << std::endl;
  std::cout << "Right Front Encoder: " << GetRightDistance().value() << std::endl;
  //std::cout << "RAW ------ Left Front Encoder: " << m_leftFrontEncoder.GetPosition() << std::endl;
  //std::cout << "RAW ------ Right Front Encoder: " << m_rightFrontEncoder.GetPosition() << std::endl;
}

units::meter_t Drivebase::GetAverageEncoderDistance() {
  //unnecessary
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

/*void Drivebase::SetMaxOutput(double maxOutput) {
  //unnecessary
  m_drive.SetMaxOutput(maxOutput);
}*/

units::degree_t Drivebase::GetHeading() const {
  //unnecessary
  return m_gyro.GetRotation2d().Degrees(); //yaw
}

double Drivebase::GetTurnRate() {
  //unnecessary
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

  //TRYING TO REPLICATE DRIVEBASE
  return frc::DifferentialDriveWheelSpeeds{GetLeftVelocity(),
                                           GetRightVelocity()};
  //return {GetLeftVelocity(),GetRightVelocity()};
}

void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  //Test
  //m_odometry.ResetPosition(0_deg, 0_m, 0_m, pose);
  m_odometry.ResetPosition(m_gyro.GetRotation2d(), 0_m, 0_m, pose);
  /*TRYING TO REPLICATE DRIVEBASE
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),GetLeftDistance(), GetRightDistance(), pose);
  std::cout << "X Position: " << m_odometry.GetPose().X().value() << " Y Position: " << m_odometry.GetPose().Y().value() << std::endl;
  */
}



