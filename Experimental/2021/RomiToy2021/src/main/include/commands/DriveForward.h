// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <cmath>

#include "subsystems/Drivetrain.h"

/**
 * A command supporting forward motion at a fixed speed (until interrupted) that
 * uses PID control to try to correct for errors (detected via either a gyro or
 * the encoders).
 *
 * This class is based on discussion at
 * https://frc-pdr.readthedocs.io/en/latest/control/driving_straight.html.
 */
class DriveForward
    : public frc2::CommandHelper<frc2::CommandBase, DriveForward>
{
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
 DriveForward(
     Drivetrain *driveTrain, double power,
     std::function<bool()> stopCondition = [] { return false; },
     SensorMode sensorMode = Encoders, bool noisy = false)
     : m_driveTrain(driveTrain),
       m_power(std::max(0.0, std::min(power, 1.0))),
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

 void End(bool interrupted) override;

private:
 double GetSteeringError();

 //
 // Data members
private:
 Drivetrain *const m_driveTrain;
 const double m_power;
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
 const double kD = 0;  ///< derivative gain (can be 0, if we're just doing P/PI)

 const double kCurveInterval = 0.02;  ///< period over which we're sampling
                                      ///< error (0.02sec, since it's 50Hz)
};
