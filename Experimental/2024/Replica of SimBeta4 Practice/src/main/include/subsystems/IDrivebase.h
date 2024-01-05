// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "utils/DeadBandEnforcer.h"


class IDrivebase : public frc2::SubsystemBase {
  // Useful class constants.
 public:
    static constexpr units::meters_per_second_t MAX_SPEED{3.0};

    static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{
      std::numbers::pi};
    
    static constexpr bool ENABLE_VOLTAGE_APPLICATION = true;


     public:
  using SimpleMotorFeedforward = frc::SimpleMotorFeedforward<units::meters>;
  using kv_unit = SimpleMotorFeedforward::kv_unit;
  using ka_unit = SimpleMotorFeedforward::ka_unit;
 
 private:

  const std::unique_ptr<frc::PIDController> m_leftPIDController;
  const std::unique_ptr<frc::PIDController> m_rightPIDController;

  const std::unique_ptr<SimpleMotorFeedforward> m_feedforward;

  const frc::DifferentialDriveKinematics m_kinematics;

  private:
  //logging data to dashboard

  static bool m_logWheelSpeedData;

  //multiple constructors for versatility ig

  public:

  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV)
      : IDrivebase(trackWidth, kP, kI, kD, kS, kV,
                   units::unit_t<ka_unit>(0.0)) {
  }


  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV,
             units::unit_t<ka_unit> kA)
      : m_leftPIDController(new frc::PIDController(kP, kI, kD)),
        m_rightPIDController(new frc::PIDController(kP, kI, kD)),
        m_feedforward(
            new frc::SimpleMotorFeedforward<units::meters>(kS, kV, kA)),
        m_kinematics(trackWidth) {
  }

  //added thingy thats hidden usually I think. Calle a destructor. 

  virtual ~IDrivebase() = default;

  //implementation of arcade drive and it sends the stuff to the smart dashboard. See lower comment

    void arcadeDrive(units::meters_per_second_t xSpeed,
                   units::radians_per_second_t rot) {
    logValue("xSpeed", xSpeed.value());
    logValue("rotSpeed", rot.value());


    //Seems like what this is doing is just converting the numbers into types that can be used

    frc::ChassisSpeeds speeds;
    speeds.vx = xSpeed;
    speeds.omega = rot;
    const auto wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);

    setSpeeds(wheelSpeeds);
  }

 void stop() {
    arcadeDrive(0_mps, 0_rad_per_s);
  }

    //Here are a bunch of getter methods
  
  const frc::DifferentialDriveKinematics& getKinematics() {
    return m_kinematics;
  }

  const SimpleMotorFeedforward& getMotorFeedforward() {
    return *m_feedforward;
  }

  double getKP() {
    return m_leftPIDController->GetP();
  }

  double getKI() {
    return m_leftPIDController->GetI();
  }

  double getKD() {
    return m_leftPIDController->GetD();
  }

  void setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);

 frc::DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return frc::DifferentialDriveWheelSpeeds{getLeftEncoder().getVelocity(),
                                             getRightEncoder().getVelocity()};
  }


  void updateOdometry() {
    getOdometry().Update(getGyro().getRotation2d(),
                         getLeftEncoder().getPosition(),
                         getRightEncoder().getPosition());
  }

  frc::Pose2d getPose() {
    return getOdometry().GetPose();
  }

  //I believe its virtual because it will be overwritten in one of the inheriting classes

  virtual void resetOdometry(frc::Pose2d pose) {
    getLeftEncoder().reset();
    getRightEncoder().reset();
    getOdometry().ResetPosition(getGyro().getRotation2d(),
                                getLeftEncoder().getPosition(),
                                getRightEncoder().getPosition(), pose);
  }

   public:

   //And these are just things to help show the data better. Probably more geared towards simulation

  static void enableLogging(bool tf) {
    m_logWheelSpeedData = tf;
  }

  static bool isLoggingEnabled() {
    return m_logWheelSpeedData;
  }

  static void logValue(std::string_view label, double val) {
    if (m_logWheelSpeedData) {
      frc::SmartDashboard::PutNumber(label, val);
    }
  }

   public:

  virtual void Periodic();

  protected:
  //stuff that we dont want tampered with

    void setMotorVoltages(units::volt_t leftPower, units::volt_t rightPower);

  static double convertVoltageToPercentSpeed(units::volt_t volts) {
    const double inputVoltage = frc::RobotController::GetInputVoltage();
    const double mps = (volts.value() / inputVoltage);
    const double speedPercentage =
        m_voltageDeadbandEnforcer(mps / MAX_SPEED.value());
    return speedPercentage;
  }

   protected:

   //remmeber virtual means that they need to be overwritten. This is just saying that yes they will exsist.

   virtual void setMotorVoltagesImpl(units::volt_t leftPower,
                                    units::volt_t rightPower) = 0;

  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;

  virtual TrivialEncoder& getRightEncoder() = 0;

  virtual IGyro& getGyro() = 0;


 private:
  units::volt_t m_lastLeftVoltage{0}, m_lastRightVoltage{0};
  static DeadBandEnforcer m_voltageDeadbandEnforcer;
};
