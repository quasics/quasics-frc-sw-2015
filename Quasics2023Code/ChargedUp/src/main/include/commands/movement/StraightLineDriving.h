// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StraightLineDriving
    : public frc2::CommandHelper<frc2::CommandBase, StraightLineDriving> {
 public:
  StraightLineDriving(Drivebase* drivebase, double speed,
                      units::meter_t distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* const m_drivebase;
  const double m_speed;
  const units::meter_t m_distance;

  frc2::PIDController pid{StraightDrivingConstants::PID::kP,
                          StraightDrivingConstants::PID::kI,
                          StraightDrivingConstants::PID::kD};

  frc::SlewRateLimiter<units::scalar> SlewRateLimiter{0.5 / 1_s};
  bool accelerating = true;

  units::degree_t originalAngle = 0_deg;
  units::degree_t currentAngle = 0_deg;
  double robotSpeed = 0;
  units::meter_t originalDistance = 0_m;
  units::meter_t currentDistance = 0_m;
  units::meter_t distanceToDestination = 0_m;
  double subtraction = 0;
  double gradualreduction = 0.5;
  int counter = 0;
};
