// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunningLights.h"

const units::second_t RunningLights::STEP_TIME = 0.05_s;
const int RunningLights::PULSE_SIZE = 5;
const frc::AddressableLED::LEDData RunningLights::PULSE_COLOR = Lights::GREEN;

RunningLights::RunningLights(Lights* lights) : m_lights(lights) {
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
  if (!m_timer.HasElapsed(STEP_TIME)) {
    return;
  }
  const int pulseStartPosition = (m_lastPos + 1) % m_lights->GetStripLength();

  m_lights->SetStripColor([pulseStartPosition](int position) {
    // TODO: Handle wrap-around
    if (position >= pulseStartPosition &&
        position < (pulseStartPosition + PULSE_SIZE)) {
      return PULSE_COLOR;
    }
    return Lights::BLACK;
  });
  m_lastPos = pulseStartPosition;
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
