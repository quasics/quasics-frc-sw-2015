// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * TODO: Add actual docs for this command....
 */
class Wait : public frc2::CommandHelper<frc2::Command, Wait> {
 public:
  Wait(units::second_t time);

  void Initialize() override;

  bool IsFinished() override;

 private:
  const units::second_t m_time;
  frc::Timer m_stopWatch;
};
