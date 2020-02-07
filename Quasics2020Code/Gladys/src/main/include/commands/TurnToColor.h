/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "subsystems/CommandPanel.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
/**
 * A command that turns to a color.
 */
class TurnToColor
    : public frc2::CommandHelper<frc2::CommandBase, TurnToColor> {
 public:
  TurnToColor(CommandPanel*controlPanel, CommandPanel::Color aimColor);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  CommandPanel*m_controlPanel;
  CommandPanel::Color m_aimColor;
};
