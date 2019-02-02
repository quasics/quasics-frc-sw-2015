/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class ElevatorToTop : public frc::Command {
 public:
  ElevatorToTop();
  void Initialize() override;
  void Execute() override;  // CODE_REVIEW (mjh): This function isn't actually
                            // needed for this class.
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};
