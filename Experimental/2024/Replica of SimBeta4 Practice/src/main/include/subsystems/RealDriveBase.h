#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "sensors/OffsetGyro.h"
#include "subsystems/IDrivebase.h"

class RealDriveBase : public IDrivebase {
 public:
  RealDriveBase();

  void Periodic() override {
    IDrivebase::Periodic();
  }

//overwridding all of the generic functions created in the parent Idrivebase class

 protected:
  void setMotorVoltagesImpl(units::volt_t leftPower,
                            units::volt_t rightPower) override;

  frc::DifferentialDriveOdometry& getOdometry() override {
    return m_odometry;
  }

  TrivialEncoder& getLeftEncoder() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder() override {
    return *m_rightTrivialEncoder;
  }

  IGyro& getGyro() override {
    return m_offsetGyro;
  }


//this should look identical to the code we have on all our previous years

 private:
  void configureEncoders();

  void resetEncoders() {
    m_leftFrontEncoder.SetPosition(0);
    m_leftBackEncoder.SetPosition(0);
    m_rightFrontEncoder.SetPosition(0);
    m_rightBackEncoder.SetPosition(0);
  }

 private:
  rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftBackEncoder = m_leftBack.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightBackEncoder = m_rightBack.GetEncoder();

  frc::MotorControllerGroup m_leftSide{m_leftFront, m_leftBack};
  frc::MotorControllerGroup m_rightSide{m_rightFront, m_rightBack};

  frc::ADXRS450_Gyro m_realGyro{frc::SPI::Port::kOnboardCS0};

  frc::DifferentialDriveOdometry m_odometry;


 private:
  //wraps it into generic gyro so we can use it

  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_realGyro)};

  //This is the thing I need to ask about
  OffsetGyro m_offsetGyro{*m_trivialGyro};

  
  //common interface thingy
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftFrontEncoder)};

  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightFrontEncoder)};
};

