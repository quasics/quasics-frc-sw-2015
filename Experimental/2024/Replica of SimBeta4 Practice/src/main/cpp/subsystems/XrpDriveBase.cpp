#include "subsystems/XrpDriveBase.h"

#include "utils/DeadBandEnforcer.h"

namespace {
  constexpr double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0);  
  constexpr double kCountsPerMotorShaftRev = 12.0;
  constexpr double kCountsPerRevolution =
      kCountsPerMotorShaftRev * kGearRatio; 
  constexpr double kWheelDiameterMeters = 0.060;

//more sample ones

  constexpr double kP = 8.5;
  constexpr double kI = 0;
  constexpr double kD = 0;

  constexpr auto kS = 1_V;
  constexpr auto kV = 3 * (1_V * 1_s / 1_m);
  constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);
  constexpr auto kTrackWidth = 0.155_m;
}  // namespace

//this is a very similar implementation to the RealDriveBase.cpp

XrpDriveBase::XrpDriveBase()
    : IDrivebase(kTrackWidth, kP, kI, kD, kS, kV, kA),
      m_odometry{0_deg, 0_m, 0_m} {

  m_rightXrpMotor.SetInverted(true);
  m_leftXrpMotor.SetInverted(false);


  m_leftXrpEncoder.SetDistancePerPulse(
      (std::numbers::pi * kWheelDiameterMeters) / kCountsPerRevolution);
  m_rightXrpEncoder.SetDistancePerPulse(
      (std::numbers::pi * kWheelDiameterMeters) / kCountsPerRevolution);

  updateOdometry();
}

void XrpDriveBase::setMotorVoltagesImpl(units::volt_t leftPower,
                                        units::volt_t rightPower) {
  m_leftXrpMotor.Set(convertVoltageToPercentSpeed(leftPower));
  m_rightXrpMotor.Set(convertVoltageToPercentSpeed(rightPower));
  m_leftXrpMotor.SetVoltage(leftPower);
  m_rightXrpMotor.SetVoltage(rightPower);
}