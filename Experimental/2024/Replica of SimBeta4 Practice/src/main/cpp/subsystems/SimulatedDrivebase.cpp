#include "subsystems/SimulatedDriveBase.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

//these need to be changed. Just sample

constexpr auto kS = 1_V;
constexpr auto kV = 3 * (1_V * 1_s / 1_m);
constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);

constexpr double kP = 8.5;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::meter_t SIMULATED_TRACK_WIDTH_METERS = 0.381_m * 2;

//remmeber that most of this is already defined in the .h so there is no need to redefine

SimulatedDriveBase::SimulatedDriveBase()
    : IDrivebase(SIMULATED_TRACK_WIDTH_METERS, kP, kI, kD, kS, kV, kA),
      m_odometry{frc::Rotation2d(), units::meter_t(0), units::meter_t(0)} {
  SetName("SimulatedDriveBase");
  m_gyro.Reset();

  m_rightMotor.SetInverted(true);


  m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
                                    kEncoderResolution);
  m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
                                     kEncoderResolution);

  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  m_rightMotor.SetInverted(true);

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

void SimulatedDriveBase::SimulationPeriodic() {

//and this is just the stuff used to make the simulator work

  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftMotor.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{m_rightMotor.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees().value());
}
