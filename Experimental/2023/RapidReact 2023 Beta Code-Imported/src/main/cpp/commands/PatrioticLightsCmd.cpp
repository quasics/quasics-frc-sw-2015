// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PatrioticLightsCmd.h"

PatrioticLightsCmd::PatrioticLightsCmd(Lighting* lighting,
                                       const units::second_t cycleTime,
                                       bool lightsMoveDown)
    : m_lighting(lighting),
      m_lastStartingPosition(0),
      m_timerInterval(cycleTime / lighting->GetNumberOfLEDs()),
      m_lightsMoveDown(lightsMoveDown) {
  AddRequirements(lighting);
}

// Called when the command is initially scheduled.
void PatrioticLightsCmd::Initialize() {
  m_lastStartingPosition = 0;
  m_lighting->SetEachCellToColor([this](int pos) {
    const int totalLights = m_lighting->GetNumberOfLEDs();
    const int slotsForColor = totalLights / 3;
    if (pos <= slotsForColor) {
      return frc::Color::kRed;
    } else if (pos <= (slotsForColor * 2)) {
      return frc::Color::kWhite;
    } else {
      return frc::Color::kBlue;
    }
  });
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void PatrioticLightsCmd::Execute() {
  if (m_timer.HasElapsed(m_timerInterval)) {
    const auto stripSize = m_lighting->GetNumberOfLEDs();
    const int slotsForColor = stripSize / 3;

    m_lastStartingPosition = (m_lastStartingPosition + 1) % stripSize;
    m_lighting->SetEachCellToColor([this, stripSize, slotsForColor](int pos) {
      auto effectivePos =
          m_lightsMoveDown
              ? ((pos + m_lastStartingPosition + stripSize) % stripSize)
              : ((pos - m_lastStartingPosition + stripSize) % stripSize);
      if (effectivePos <= slotsForColor) {
        return frc::Color::kRed;
      } else if (effectivePos <= (slotsForColor * 2)) {
        return frc::Color::kWhite;
      } else {
        return frc::Color::kBlue;
      }
    });
    m_timer.Reset();
  }
}

// Called once the command ends or is interrupted.
void PatrioticLightsCmd::End(bool interrupted) {
}
