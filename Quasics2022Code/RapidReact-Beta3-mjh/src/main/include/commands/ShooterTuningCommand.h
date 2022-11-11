// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableEntry.h>
#include <subsystems/Shooter.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterTuningCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShooterTuningCommand> {
 public:
  ShooterTuningCommand(Shooter* shooter,
                       double initialShooterSpeedPercent = 0.0,
                       double initialRollerSpeedPercent = 0.0);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void UpdateSpeeds();

 private:
  Shooter* const m_shooter;
  nt::NetworkTableEntry m_shooterSpeedSlider;
  nt::NetworkTableEntry m_rollerSpeedSlider;
};
