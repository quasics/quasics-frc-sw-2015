// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunningLights.h"

const units::second_t RunningLights::DEFAULT_STEP_TIME = 0.05_s;
const int RunningLights::DEFAULT_PULSE_SIZE = 5;
const frc::AddressableLED::LEDData RunningLights::DEFAULT_PULSE_COLOR =
    Lights::GREEN;

RunningLights::RunningLights(Lights* lights, units::second_t stepTime,
                             int pulseSize,
                             frc::AddressableLED::LEDData pulseColor)
    : m_lights(lights),
      m_stepTime(stepTime),
      m_pulseSize(pulseSize),
      m_pulseColor(pulseColor) {
  AddRequirements(m_lights);
}

// Called when the command is initially scheduled.
void RunningLights::Initialize() {
  if (m_lights == nullptr) return;

  m_lights->SetStripColor(0, 0, 0);
  m_lastPos = 0;
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RunningLights::Execute() {
  if (m_lights == nullptr) {
    return;
  }
  if (!m_timer.HasElapsed(m_stepTime)) {
    return;
  }
  const int pulseStartPosition = (m_lastPos + 1) % m_lights->GetStripLength();

  m_lights->SetStripColor([this, pulseStartPosition](int position) {
    // TODO: Handle wrap-around
    if (position >= pulseStartPosition &&
        position < (pulseStartPosition + this->m_pulseSize)) {
      return this->m_pulseColor;
    }
    return Lights::BLACK;
  });
  m_lastPos = pulseStartPosition;
  m_timer.Reset();
}

// Called once the command ends or is interrupted.
void RunningLights::End(bool interrupted) {
  m_timer.Stop();
  if (m_lights != nullptr) {
    m_lights->SetStripColor(0, 100, 0);
  }
}

// Returns true when the command should end.
bool RunningLights::IsFinished() { return false; }
