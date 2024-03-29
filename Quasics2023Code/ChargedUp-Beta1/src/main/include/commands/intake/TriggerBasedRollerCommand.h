// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ConfigSettings.h"
#include "subsystems/IntakeRoller.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * CODE_REVIEW(matthew): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.
 */
class TriggerBasedRollerCommand
    : public frc2::CommandHelper<frc2::Command, TriggerBasedRollerCommand> {
 public:
  TriggerBasedRollerCommand(IntakeRoller* intakeRoller,
                            ConfigSettings* settings,
                            std::function<bool()> intakeTriggered,
                            std::function<bool()> exhaustTriggered);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  IntakeRoller* const m_intakeRoller;
  ConfigSettings* m_settings;
  std::function<bool()> m_intakeTriggered;
  std::function<bool()> m_exhaustTriggered;
};
