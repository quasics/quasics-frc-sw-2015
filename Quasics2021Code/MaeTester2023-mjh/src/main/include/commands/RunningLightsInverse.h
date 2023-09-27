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
class RunningLightsInverse
    : public frc2::CommandHelper<frc2::CommandBase, RunningLightsInverse> {
 public:
  static const units::second_t DEFAULT_STEP_TIME;
  static const int DEFAULT_PULSE_SIZE;
  static const frc::AddressableLED::LEDData DEFAULT_PULSE_COLOR;

 private:
  Lights* const m_lights;
  const units::second_t m_stepTime;
  const int m_pulseSize;
  const frc::AddressableLED::LEDData m_pulseColor;

  frc::Timer m_timer;
  int m_lastPos;

 public:
  RunningLightsInverse(Lights* lights)
      : RunningLightsInverse(lights, DEFAULT_STEP_TIME, DEFAULT_PULSE_SIZE,
                      DEFAULT_PULSE_COLOR) {}
  RunningLightsInverse(Lights* lights, units::second_t stepTime, int pulseSize,
                frc::AddressableLED::LEDData pulseColor);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};