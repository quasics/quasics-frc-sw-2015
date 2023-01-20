// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PatrioticLightsCmd
    : public frc2::CommandHelper<frc2::CommandBase, PatrioticLightsCmd> {
 public:
  PatrioticLightsCmd(Lighting* lighting,
                     const units::second_t cycleTime = 3.0_s,
                     bool lightsMoveDown = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Lighting* const m_lighting;
  int m_lastStartingPosition;
  frc::Timer m_timer;
  const units::second_t m_timerInterval;
  const bool m_lightsMoveDown;
};
