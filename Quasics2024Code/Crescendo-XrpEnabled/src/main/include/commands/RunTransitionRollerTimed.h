// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TransitionRoller.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunTransitionRollerTimed
    : public frc2::CommandHelper<frc2::Command, RunTransitionRollerTimed> {
 public:
  RunTransitionRollerTimed(TransitionRoller &transitionRoller,
                           double transitionSpeed, units::second_t time,
                           bool transitionTakingIn);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  TransitionRoller &m_transitionRoller;
  const double m_transitionSpeed;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
  const bool m_transitionTakingIn;
};