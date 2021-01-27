// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CommonDriveSubsystem.h"

/**
 * A sample implementation of a "tank drive" command, for use in teleop mode.
 */
class TeleopTankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TeleopTankDrive> {
 public:
  TeleopTankDrive(CommonDriveSubsystem* driveBase,
                  std::function<double()> leftSpeedSupplier,
                  std::function<double()> rightSpeedSupplier)
      : m_driveBase(driveBase),
        m_leftSpeedSupplier(leftSpeedSupplier),
        m_rightSpeedSupplier(rightSpeedSupplier) {
    AddRequirements({m_driveBase});
  }

  void Execute() override {
    m_driveBase->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
  }

  void End(bool interrupted) override {
    m_driveBase->Stop();
  }

 private:
  CommonDriveSubsystem* const m_driveBase;
  std::function<double()> m_leftSpeedSupplier;
  std::function<double()> m_rightSpeedSupplier;
};
