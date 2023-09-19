// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lights.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunningLights
    : public frc2::CommandHelper<frc2::CommandBase, RunningLights> {
 private:
  Lights* m_lights;
  frc::Timer m_timer;
  int m_lastPos;
  static const units::second_t STEP_TIME;
  static const int PULSE_SIZE;
  static const frc::AddressableLED::LEDData PULSE_COLOR;

 public:
  RunningLights(Lights* lights);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
