// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/**
 * Tank drives using the speeds returned by leftSpeedFunction() and
 * rightSpeedFunction(). Used by joysticks in RobotContainer
 *
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(Drivebase* drivebase, std::function<double()> leftSpeedFunction,
            std::function<double()> rightSpeedFunction);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void UpdateSpeeds();

  Drivebase* m_drivebase;
  std::function<double()> m_leftSpeedFunction;
  std::function<double()> m_rightSpeedFunction;

  // working with logitech controller
};
