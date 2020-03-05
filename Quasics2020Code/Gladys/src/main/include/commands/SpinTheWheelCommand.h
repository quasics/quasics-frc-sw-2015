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

/// TODO(RJ): Document this class (missing class-level JavaDoc-style comments
// describing its purpose).
class SpinTheWheelCommand
    : public frc2::CommandHelper<frc2::CommandBase, SpinTheWheelCommand> {
 public:
  /**
   * adds the dependancy to the ControlPanel and initializes the controlPanel
   *and direction
   * @param controlPanel used by the big red button to stop everything
   * @param inverted inverts the direction
   **/
  SpinTheWheelCommand(ControlPanel* controlPanel, bool inverted);
  /**
   * turns the motor on
   **/
  void Initialize() override;
  /**
   * this method ends the turning of the motor.
   * @param interrupted used by the big red button to stop everything
   **/
  void End(bool interrupted) override;
  // make sure it runs
  bool IsFinished() override;

 private:
  // ControlPanel object
  ControlPanel* m_controlPanel;
  // direction
  bool f;
};
