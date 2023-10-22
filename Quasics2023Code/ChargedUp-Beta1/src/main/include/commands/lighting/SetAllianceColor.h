// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * Takes the input from the Field Management System and turns the robot light
 * strips to that color.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetAllianceColor
    : public frc2::CommandHelper<frc2::Command, SetAllianceColor> {
 public:
  SetAllianceColor(Lighting* lighting);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Lighting* m_lighting;
};
