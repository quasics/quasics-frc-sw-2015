// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Drivebase.h>

/**
 * Command providing support for "tank drive" operations during teleop mode.
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  /**
   * Constructor.
   *
   * @param drivebase   pointer to the Drivebase subsystem being used
   * @param driverStick pointer to the Logitech controller used by the driver
   */
  TankDrive(Drivebase *drivebase, frc::Joystick *driverStick);

  // Methods defined for Commands, which we're overriding.
 public:
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  // Private utility functions
 private:
  /**
   * Updates the current left/right motor speeds on the drive base, in response
   * to the joystick positions.
   */
  void UpdateSpeeds();

  // Data members
 private:
  /** Drive base we're working with. */
  Drivebase *m_drivebase;

  /** Logitech controller providing feedback for speeds. */
  frc::Joystick *m_driverStick;
};
