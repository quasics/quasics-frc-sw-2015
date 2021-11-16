#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/math.h>

#include <cmath>
#include <iostream>

#include "CommonDriveSubsystem.h"

/**
 * A command supporting forward motion at a variable speed (until interrupted)
 * that uses PID control to try to correct for errors (detected via either a
 * gyro or the encoders).
 *
 * This class is based on discussion at
 * https://frc-pdr.readthedocs.io/en/latest/control/driving_straight.html.
 */
class DriveStraightCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveStraightCommand> {
 public:
  /**
   * Different sensors we can use to detect deviation from straight-line
   * movement.
   */
  enum SensorMode { Gyro, Encoders };

  /**
   * @param driveTrain pointer to the drive train substem
   * @param power power to be applied (clamped to the range [0..1])
   * @param stopCondition used to signal when movement should stop (e.g.,
   *                      after a specific distance has been traveled, time
   *                      elapsed, etc.)
   * @param sensorMode specifies the type of sensor to be used to detect errors
   *                   in direction
   * @param noisy if true, log debugging output while the command is running
   */
  DriveStraightCommand(
      CommonDriveSubsystem *driveTrain, std::function<double()> powerCondition,
      std::function<bool()> stopCondition = [] { return false; },
      SensorMode sensorMode = Encoders, bool noisy = false)
      : m_driveTrain(driveTrain),
        m_powerCondition(powerCondition),
        m_stopCondition(stopCondition),
        m_useGyro(sensorMode == Gyro),
        m_noisy(noisy) {
    AddRequirements({m_driveTrain});
  }

  //
  // Overridden methods from base class.
 public:
  void Initialize() override;

  // Note: at each iteration, we will compute error (from heading, or delta in
  // left/right distance), then apply correction factor:
  //    correction = (kP * error)
  //                 + (kI * m_integral)
  //                 + (kD * change in error)
  void Execute() override;

  bool IsFinished() override {
    return m_stopCondition();
  }

  void End(bool interrupted) override {
    m_driveTrain->Stop();
  }

 private:
  double GetSteeringError();

  //
  // Data members
 private:
  CommonDriveSubsystem *const m_driveTrain;
  std::function<double()> m_powerCondition;
  std::function<bool()> m_stopCondition;
  const bool m_useGyro;
  const bool m_noisy;

  double m_integral = 0;   ///< cumulative error sample (should trend to 0)
  double m_lastError = 0;  ///< used to calculate change in error
                           ///< between samples

  // These values should really be tuned to the robot, per discussion cited
  // above.  However, these seem to work well enough for now.
  const double kP = 1;  ///< proportional gain (should always be >0)
  const double kI = 1;  ///< integral gain (can be 0, if we're just doing P)
  const double kD =
      0;  ///< derivative gain (can be 0, if we're just doing P/PI)

  const double kCurveInterval = 0.02;  ///< period over which we're sampling
                                       ///< error (0.02sec, since it's 50Hz)
};

inline void DriveStraightCommand::Initialize() {
  if (m_useGyro) {
    m_driveTrain->ResetGyro();
  } else {
    m_driveTrain->ResetEncoders();
  }
  m_integral = 0;
  m_lastError = 0;
}

inline double DriveStraightCommand::GetSteeringError() {
  double error;
  if (m_useGyro) {
    const double rawAngle = m_driveTrain->GetZAxisGyro().GetAngle();
    const double clampedAngle = double(int(rawAngle) % 360);
    const double adjustedAngle =
        (clampedAngle <= 180) ? clampedAngle : clampedAngle - 360;
    error = -(adjustedAngle / 180.0);  // our target delta for the angle is zero
    if (m_noisy) {
      std::cout << "rawAngle: " << rawAngle << ", clamped: " << clampedAngle
                << ", adjusted: " << adjustedAngle;
    }
  } else {
    const auto distanceLeft = units::math::abs(m_driveTrain->GetLeftDistance());
    const auto distanceRight =
        units::math::abs(m_driveTrain->GetRightDistance());
    error =
        -double(distanceLeft - distanceRight);  // our target delta for distance
                                                // between the two sides is zero
    if (m_noisy) {
      std::cout << "distanceLeft: " << distanceLeft
                << ", distanceRight: " << distanceRight;
    }
  }
  return error;
}

// Called repeatedly when this Command is scheduled to run
inline void DriveStraightCommand::Execute() {
  const double power = std::max(0.0, std::min(m_powerCondition(), 1.0));
  if(m_noisy) {
    std::cout << "Power: " << power;
  }

  const double error = GetSteeringError();
  m_integral += (error * kCurveInterval);
  double derivative = ((error - m_lastError) / kCurveInterval);

  const double turnPower = (kP * error)          // Proportional
                           + (kI * m_integral)   // Integral
                           + (kD * derivative);  // Derivative

  if (m_noisy) {
    std::cout << ", error: " << error << ", integral: " << m_integral
              << ", derivative: " << derivative << ", turnPower: " << turnPower
              << std::endl;
  }

  m_driveTrain->ArcadeDrive(power, turnPower, false);
}
