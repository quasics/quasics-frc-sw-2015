// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingAllianceLights.h"

BreathingAllianceLights::BreathingAllianceLights(Lighting* lights,
                                                 double maxIntensity)
    : m_lighting(lights), m_maxIntensityPercent(maxIntensity) {
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void BreathingAllianceLights::Initialize() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
    m_ColorAlliance = ColorAlliance::kRed;
  } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
    m_ColorAlliance = ColorAlliance::kBlue;
  } else {
    m_ColorAlliance = ColorAlliance::kInvalid;
  }
  // (Matthew): There actually *is* a 3rd case in the alliance values.
  // It might make sense to explicitly handle that, rather than just always
  // falling back on "It's not the red alliance, so it must be blue", without
  // any indication that something is funky.(done)

  // Reset the intensity before starting.
  m_currentIntensityPercent = 0;

  // OK, set the lights to initial intensity/color.
  if (m_ColorAlliance == ColorAlliance::kRed) {
    m_lighting->SetAllToColor(fullIntensity * m_currentIntensityPercent, 0, 0);
  } else if (m_ColorAlliance == ColorAlliance::kBlue) {
    m_lighting->SetAllToColor(0, 0, fullIntensity * m_currentIntensityPercent);
  } else {
    m_lighting->SetAllToColor(fullIntensity * m_currentIntensityPercent, 0,
                              fullIntensity * m_currentIntensityPercent);
  }
}

// Called repeatedly when this Command is scheduled to run
void BreathingAllianceLights::Execute() {
  if (m_currentIntensityPercent >= m_maxIntensityPercent) {
    m_increment = -0.01;
    m_currentIntensityPercent = m_maxIntensityPercent;
  }
  if (m_currentIntensityPercent <= 0) {
    m_increment = 0.01;
    m_currentIntensityPercent = 0;
  }

  if (m_ColorAlliance == ColorAlliance::kRed) {
    m_lighting->SetAllToColor(fullIntensity * m_currentIntensityPercent, 0, 0);
  } else if (m_ColorAlliance == ColorAlliance::kBlue) {
    m_lighting->SetAllToColor(0, 0, fullIntensity * m_currentIntensityPercent);
  } else {
    m_lighting->SetAllToColor(fullIntensity * m_currentIntensityPercent, 0,
                              fullIntensity * m_currentIntensityPercent);
  }
  m_currentIntensityPercent += m_increment;
}

// Called once the command ends or is interrupted.
void BreathingAllianceLights::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}
