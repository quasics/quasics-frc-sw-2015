// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeRoller.h"

// TODO: (CODE_REVIEW) Add comments.
class TriggerBasedIntaking
    : public frc2::CommandHelper<frc2::Command, TriggerBasedIntaking> {
 public:
  TriggerBasedIntaking(IntakeRoller& intake, frc::Joystick* driverController);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  IntakeRoller& m_intake;
  frc::Joystick* m_driverController;
};
