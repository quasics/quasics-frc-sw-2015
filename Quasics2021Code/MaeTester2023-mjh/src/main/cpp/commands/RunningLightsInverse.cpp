// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunningLightsInverse.h"
#include <iostream>

const units::second_t RunningLightsInverse::DEFAULT_STEP_TIME = 0.05_s;
const int RunningLightsInverse::DEFAULT_PULSE_SIZE = 5;
const frc::AddressableLED::LEDData RunningLightsInverse::DEFAULT_PULSE_COLOR =
    Lights::GREEN;

RunningLightsInverse::RunningLightsInverse(Lights* lights, units::second_t stepTime,
                             int pulseSize,
                             frc::AddressableLED::LEDData pulseColor)
    : m_lights(lights),
      m_stepTime(stepTime),
      m_pulseSize(pulseSize),
      m_pulseColor(pulseColor) {
  AddRequirements(m_lights);
}

// Called when the command is initially scheduled.
void RunningLightsInverse::Initialize() {
  if (m_lights == nullptr) return;

  m_lights->SetStripColor(0, 0, 0);
  m_lastPos = m_lights->GetStripLength();
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RunningLightsInverse::Execute() {
  if (m_lights == nullptr) {
    return;
  }
  if (!m_timer.HasElapsed(m_stepTime)) {
    return;
  }

    // Position where the pulse starts on the LED strip.
  const int pulseStartPosition = (m_lastPos - 1 + m_lights->GetStripLength()) % m_lights->GetStripLength();
  // Note: this is the first position *after* the pulse ends.
  const int pulseEndPosition =
      (m_lastPos + 1 + m_pulseSize) % m_lights->GetStripLength();

  // Light 'em up!
  m_lights->SetStripColor(
      [this, pulseStartPosition, pulseEndPosition](int position) {
        if (pulseStartPosition <= pulseEndPosition) {
          // Simple case: turning on lights from [startPos,endPos)
          if (position >= pulseStartPosition &&
              position < (pulseStartPosition + this->m_pulseSize)) {
            return this->m_pulseColor;
          }
        } else {
          // Handles wrap-around: turn on lights from [0,endPos] and
          // [startPos,...]
          if (position >= pulseStartPosition || position < pulseEndPosition) {
            return this->m_pulseColor;
          }
        }

        // All other lights should be off.
        return Lights::BLACK;
      });


  // Update data used for the next pulse.
  m_lastPos = pulseStartPosition;
  m_timer.Reset();
}

// Called once the command ends or is interrupted.
void RunningLightsInverse::End(bool interrupted) {
  m_timer.Stop();
  if (m_lights != nullptr) {
    m_lights->SetStripColor(0, 100, 0);
  }
}

// Returns true when the command should end.
bool RunningLightsInverse::IsFinished() { return false; }