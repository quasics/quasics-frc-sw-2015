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

/**
 * A command that turns the control panel to the color specified by the FMS, in
 * order to complete scoring requirements for Phase 3.
 */
class TurnControlPanelToTargetColorCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TurnControlPanelToTargetColorCommand> {
 public:
  /**
   * Adds the CommandPanel passed in here as a dependancy
   * @param controlPanel An instance of the CommandPanel class
   * @see CommandPanel
   **/
  TurnControlPanelToTargetColorCommand(CommandPanel* controlPanel);
  /**
   * Gets target color and stops if no data is recieved. Starts the motor if
   *data is recieved and sets the target color to the color recieved from the
   *gameData.
   **/
  void Initialize() override;

  /**
   * this method ends the turning of the motor.
   * @param interrupted used by the big red button to stop everything
   **/
  void End(bool interrupted) override;
  /**
   * Checks if the command is finished by checking
   * if a color is seen.
   **/
  bool IsFinished() override;

 private:
  CommandPanel* m_controlPanel;
  CommandPanel::Color m_aimColor;
};
