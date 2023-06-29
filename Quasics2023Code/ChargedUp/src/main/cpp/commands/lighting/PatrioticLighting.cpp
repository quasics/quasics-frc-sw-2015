// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/lighting/PatrioticLighting.h"

PatrioticLighting::PatrioticLighting(Lighting* lighting,
                                     const units::second_t cycleTime,
                                     bool lightsMoveDown)
    : m_lighting(lighting),
      m_timerInterval(cycleTime / lighting->GetNumberOfLEDs()),
      m_lightsMoveDown(lightsMoveDown),
      m_lastStartingPosition(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(lighting);
}

// Called when the command is initially scheduled.
void PatrioticLighting::Initialize() {
  m_lastStartingPosition = 0;
  m_lighting->SetLightColors([this](int pos) {
    const int totalLights = m_lighting->GetNumberOfLEDs();
    const int slotsForColor = totalLights / 3;
    frc::AddressableLED::LEDData result;
    if (pos <= slotsForColor) {
      return Lighting::RED;
    } else if (pos <= (slotsForColor * 2)) {
      return Lighting::WHITE;
    } else {
      return Lighting::BLUE;
    }
  });
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void PatrioticLighting::Execute() {
  if (m_timer.HasElapsed(m_timerInterval)) {
    const auto stripSize = m_lighting->GetNumberOfLEDs();
    const int slotsForColor = stripSize / 3;

    m_lastStartingPosition = (m_lastStartingPosition + 1) % stripSize;
    m_lighting->SetLightColors([this, stripSize, slotsForColor](int pos) {
      auto effectivePos =
          m_lightsMoveDown
              ? ((pos + m_lastStartingPosition + stripSize) % stripSize)
              : ((pos - m_lastStartingPosition + stripSize) % stripSize);
      if (effectivePos <= slotsForColor) {
        return Lighting::RED;
      } else if (effectivePos <= (slotsForColor * 2)) {
        return Lighting::WHITE;
      } else {
        return Lighting::BLUE;
      }
    });
    m_timer.Reset();
  }
}

// Called once the command ends or is interrupted.
void PatrioticLighting::End(bool interrupted) {
  // Just freeze in the current positions.
}

// Returns true when the command should end.
bool PatrioticLighting::IsFinished() {
  return false;
}
