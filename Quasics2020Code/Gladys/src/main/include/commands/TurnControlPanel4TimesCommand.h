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
 * A command that turns the commandpanel 3 and a half times.
 */
class TurnControlPanel4TimesCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TurnControlPanel4TimesCommand> {
 public:
    /**
  * Adds the CommandPanel passed in here as a dependancy
  * @param controlPanel An instance of the CommandPanel class
  * @see CommandPanel
  **/
  TurnControlPanel4TimesCommand(CommandPanel* controlPanel);
   /**
  * Initialises counter, prev & currColor
  **/
  void Initialize() override; //finds initcolor


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
  CommandPanel* m_controlPanel;
  //counter
  int counter;
  //initital color sensed
  CommandPanel::Color initColor = CommandPanel::UNKNOWN,
  //previous color
                      prevColor = CommandPanel::UNKNOWN,
  //current color
                      currColor = CommandPanel::UNKNOWN;
};
