#include "subsystems/RealDriveBase.h"

using std::numbers::pi;

//sample constants so it builds from 2023
//these would usually be in the constants.h file but im guessing so that its easier to identy
//they are here

constexpr auto kS = 0.19529_V;
constexpr auto kV = 2.2329 * (1_V * 1_s / 1_m);
constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 0.29613;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::inch_t TRACK_WIDTH_METERS_SALLY = 22.0_in;

namespace RobotPhysics {
  
  constexpr double DRIVEBASE_GEAR_RATIO_MAE = 10.71;

  constexpr double DRIVEBASE_GEAR_RATIO_SALLY = 8.45;

  constexpr double DRIVEBASE_GEAR_RATIO_GLADYS = 8.45;

  constexpr double DRIVEBASE_GEAR_RATIO = DRIVEBASE_GEAR_RATIO_SALLY;


  constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;
}  // namespace RobotPhysics

//base constructor
RealDriveBase::RealDriveBase()
    : IDrivebase(TRACK_WIDTH_METERS_SALLY, kP, kI, kD, kS, kV, kA),
      m_odometry{0_deg, 0_m, 0_m} {
  SetName("RealDriveBase");

// potentially needed depending on build

  // m_leftSide.SetInverted(false);
  // m_rightSide.SetInverted(true);

  configureEncoders();

  updateOdometry();

}

//AND... this code should look very familiar

void RealDriveBase::configureEncoders() {
  const units::meter_t wheelCircumference = RobotPhysics::WHEEL_DIAMETER * pi;

  const units::meter_t gearingConversion =
      wheelCircumference / RobotPhysics::DRIVEBASE_GEAR_RATIO;

  const units::meter_t velocityCorrection = gearingConversion / 60;

  m_leftFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());


  m_leftFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

  resetEncoders();
}

void RealDriveBase::setMotorVoltagesImpl(units::volt_t leftPower,
                                         units::volt_t rightPower) {
  m_leftSide.SetVoltage(leftPower);
  m_rightSide.SetVoltage(rightPower);
}