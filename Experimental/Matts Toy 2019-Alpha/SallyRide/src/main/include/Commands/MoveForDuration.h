/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Commands/TimedCommand.h"

class MoveForDuration : public frc::TimedCommand {
 private:
  double m_power;
 public:
  explicit MoveForDuration(double timeout, double power = 0.25);
  void Initialize() override;
  void Execute() override;
  void End() override;
  void Interrupted() override;
};
