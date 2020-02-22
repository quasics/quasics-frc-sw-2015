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

// TODO(RJ): Document this class.
/**
 * A command that turns the colorwheel to a color.
 */
class TurnControlPanelToTargetColorCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TurnControlPanelToTargetColorCommand> {
 public:
  TurnControlPanelToTargetColorCommand(CommandPanel* controlPanel);

  void Initialize() override; //gets target color and stops if no data is recieved.


  void End(bool interrupted) override; //ends the search for the color by turning off the motor

  bool IsFinished() override; //looks to see if the sensor has found the color

 private:
  CommandPanel* m_controlPanel;
  CommandPanel::Color m_aimColor;
};
