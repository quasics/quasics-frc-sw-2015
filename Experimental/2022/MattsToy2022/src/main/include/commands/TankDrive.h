// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveBase.h"

/**
 * A sample implementation of a "tank drive" command, for use in teleop mode.
 *
 * TODO: Implement "switch drive".
 * TODO: Implement "turbo" and "turtle" mode.
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(DriveBase* driveBase, std::function<double()> leftPowerSupplier,
            std::function<double()> rightPowerSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  DriveBase* m_driveBase;
  std::function<double()> m_leftPowerSupplier;
  std::function<double()> m_rightPowerSupplier;
};
