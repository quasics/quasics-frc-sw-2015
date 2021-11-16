// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <algorithm>

#include "CommonDriveSubsystem.h"

/**
 * A sample implementation of a "tank drive" command, for use in teleop mode.
 *
 * This also provides support for "switch drive", where the front and rear of
 * the robot are treated as being reversed (which also means that the logical
 * left&right motors are both switched and inverted).
 */
class TeleopTankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TeleopTankDrive> {
 public:
  /**
   * Constructor.
   *
   * @param driveBase   the drive base we're working with
   * @param leftSpeedSupplier   function returning the requested speed for the
   * left side
   * @param rightSpeedSupplier  function returning the requested speed for the
   * right side
   * @param switchDriveEnabled  function returning "true" if "switch mode"
   * should be active (if not provided, switch mode is never active)
   */
  TeleopTankDrive(
      CommonDriveSubsystem* driveBase,
      std::function<double()> leftSpeedSupplier,
      std::function<double()> rightSpeedSupplier,
      std::function<bool()> switchDriveEnabled = [] { return false; })
      : m_driveBase(driveBase),
        m_leftSpeedSupplier(leftSpeedSupplier),
        m_rightSpeedSupplier(rightSpeedSupplier),
        m_switchDriveEnabled(switchDriveEnabled) {   // Bounds to [0..1]
    AddRequirements({m_driveBase});
  }

  void Execute() override {
    if (m_switchDriveEnabled()) {
      // Swap (and invert) left and right motor control
      m_driveBase->TankDrive(-m_rightSpeedSupplier(),
                             -m_leftSpeedSupplier());
    } else {
      m_driveBase->TankDrive(m_leftSpeedSupplier(),
                             m_rightSpeedSupplier());
    }
  }

  void End(bool interrupted) override { m_driveBase->Stop(); }

 private:
  CommonDriveSubsystem* const m_driveBase;
  std::function<double()> m_leftSpeedSupplier;
  std::function<double()> m_rightSpeedSupplier;
  std::function<bool()> m_switchDriveEnabled;
};
