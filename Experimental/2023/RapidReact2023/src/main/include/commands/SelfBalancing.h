// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <subsystems/Drivebase.h>
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SelfBalancing
    : public frc2::CommandHelper<frc2::CommandBase, SelfBalancing> {
 public:
  SelfBalancing(Drivebase* drivebase);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Drivebase* m_drivebase;
  frc2::PIDController pid{SelfBalancingConstants::PID::kP, SelfBalancingConstants::PID::kI, SelfBalancingConstants::PID::kD};
  double pastAngle;
  bool noFeedFowardPower = false;
  bool activatePID = false;
  double slopeOfRamp = 1;

};
