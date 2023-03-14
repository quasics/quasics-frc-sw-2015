// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
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
  StraightLineDriving(Drivebase* drivebase, frc::Joystick* driverStick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* const m_drivebase;
  frc::Joystick* const m_driverStick;

  frc2::PIDController pid{StraightDrivingConstants::PID::kP,
                          StraightDrivingConstants::PID::kI,
                          StraightDrivingConstants::PID::kD};

  units::degree_t originalAngle = 0_deg;
  units::degree_t currentAngle = 0_deg;
  double robotSpeed = 0;
};
