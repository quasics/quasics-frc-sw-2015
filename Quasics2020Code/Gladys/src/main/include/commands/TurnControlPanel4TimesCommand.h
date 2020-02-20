/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandPanel.h"

// TODO(RJ): Improve the documentation (description, at least) for this class.
/**
 * A command that turns 3 and a half times.
 */
class TurnControlPanel4TimesCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TurnControlPanel4TimesCommand> {
 public:
  TurnControlPanel4TimesCommand(CommandPanel* controlPanel);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  CommandPanel* m_controlPanel;
  int counter = 0;
  CommandPanel::Color initColor = CommandPanel::UNKNOWN,
                      prevColor = CommandPanel::UNKNOWN,
                      currColor = CommandPanel::UNKNOWN;
};
