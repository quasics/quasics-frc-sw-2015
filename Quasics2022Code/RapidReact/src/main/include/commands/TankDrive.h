// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <subsystems\Drivebase.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(Drivebase* drivebase, frc::Joystick* driverStick);

 // Methods defined for Commands, which we're overriding.
 public:
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 // Private utility functions
 private:
  void UpdateSpeeds();

 // Data members 
 private:
  Drivebase* m_drivebase;
  frc::Joystick* m_driverStick;

};
