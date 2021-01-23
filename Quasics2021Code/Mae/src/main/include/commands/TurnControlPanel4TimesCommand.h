/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ControlPanel.h"

/**
 * A command that turns the control panel between 3 and 5 times, in order to
 * complete scoring requirements in Phase 2.
 */
class TurnControlPanel4TimesCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TurnControlPanel4TimesCommand> {
 public:
  /**
   * Adds the ControlPanel passed in here as a dependancy
   * @param controlPanel An instance of the ControlPanel class
   * @see ControlPanel
   **/
  TurnControlPanel4TimesCommand(ControlPanel* controlPanel);
  /**
   * Initialises counter, prev & currColor
   **/
  void Initialize() override;  // finds initcolor

  /**
   * this method ends the turning of the motor.
   * @param interrupted used by the big red button to stop everything
   **/
  void End(bool interrupted) override;
  /**
   * Checks if the command is finished by checking
   * if a counter that keeps track of the number
   * of half turns is equal to eight.
   **/
  bool IsFinished() override;

 private:
  // Command Panel object for methods in the class
  ControlPanel* m_controlPanel;
  // counter
  int counter;
  // initital color sensed
  ControlPanel::Color initColor = ControlPanel::UNKNOWN,
                      // previous color
      prevColor = ControlPanel::UNKNOWN,
                      // current color
      currColor = ControlPanel::UNKNOWN;
};
