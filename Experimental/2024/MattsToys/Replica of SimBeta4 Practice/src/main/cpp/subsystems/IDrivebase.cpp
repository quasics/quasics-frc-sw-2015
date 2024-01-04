#include "subsystems/IDrivebase.h"

#include "utils/DeadBandEnforcer.h"

DeadBandEnforcer IDrivebase::m_voltageDeadbandEnforcer(-0.001);

namespace {
  const DeadBandEnforcer kSpeedEnforcer{0.1};
}  // namespace

bool IDrivebase::m_logWheelSpeedData{false};

void IDrivebase::Periodic() {
  updateOdometry();


//why is this needed? isn't the setMotorVoltagesImpl going to be sending the data?

  if (ENABLE_VOLTAGE_APPLICATION) {
    setMotorVoltagesImpl(m_lastLeftVoltage, m_lastRightVoltage);
  }
}

//implementation of its definition in .h

void IDrivebase::setMotorVoltages(units::volt_t leftVoltage,
                                  units::volt_t rightVoltage) {
  logValue("leftVolts", leftVoltage.value());
  logValue("rightVolts", rightVoltage.value());

  if (ENABLE_VOLTAGE_APPLICATION) {
    setMotorVoltagesImpl(leftVoltage, rightVoltage);
  }
  m_lastLeftVoltage = leftVoltage;
  m_lastRightVoltage = rightVoltage;
}

//lots of data being logged

void IDrivebase::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  logValue("leftSpeed", speeds.left.value());
  logValue("rightSpeed", speeds.right.value());


  auto leftStabilized = kSpeedEnforcer(speeds.left.value()) * 1_mps;
  auto rightStabilized = kSpeedEnforcer(speeds.right.value()) * 1_mps;
  logValue("leftStable", leftStabilized.value());
  logValue("rightStable", rightStabilized.value());

    //calculations to get desired voltages thingy

  auto leftFeedforward = m_feedforward->Calculate(leftStabilized);
  auto rightFeedforward = m_feedforward->Calculate(rightStabilized);
  logValue("leftFF", leftFeedforward.value());
  logValue("rightFF", rightFeedforward.value());

  //PID compensating so that it doesn't jump too much ig?

  double leftPidOutput = m_leftPIDController->Calculate(
      getLeftEncoder().getVelocity().value(), speeds.left.value());
  double rightPidOutput = m_rightPIDController->Calculate(
      getRightEncoder().getVelocity().value(), speeds.right.value());
  logValue("leftPid", leftPidOutput);
  logValue("rightPid", rightPidOutput);

  
  const auto leftVoltage = units::volt_t{leftPidOutput} + leftFeedforward;
  const auto rightVoltage = units::volt_t{rightPidOutput} + rightFeedforward;
  setMotorVoltages(leftVoltage, rightVoltage);
}
