
#pragma once

#include <frc/xrp/XRPGyro.h>
#include <frc/xrp/XRPMotor.h>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "subsystems/IDrivebase.h"

//the overides

class XrpDriveBase : public IDrivebase {
 public:
  XrpDriveBase();

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
    return *m_iGyro;
  }

//constants. I think this is some small practice bot?
 private:
  frc::XRPMotor m_leftXrpMotor{0};
  frc::XRPMotor m_rightXrpMotor{1};
  frc::Encoder m_leftXrpEncoder{4, 5};
  frc::Encoder m_rightXrpEncoder{6, 7};
  frc::XRPGyro m_xrpGyro;


  frc::DifferentialDriveOdometry m_odometry;

//wrapping again

  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftXrpEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightXrpEncoder)};
  std::unique_ptr<IGyro> m_iGyro{IGyro::wrapYawGyro(m_xrpGyro)};
};