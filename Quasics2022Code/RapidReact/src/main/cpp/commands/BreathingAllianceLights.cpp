// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingAllianceLights.h"

BreathingAllianceLights::BreathingAllianceLights(Lighting* lights,
                                                 double maxIntensity)
    : m_lighting(lights), maxIntensityPercent(maxIntensity) {
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void BreathingAllianceLights::Initialize() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
    isRed = true;
  } else {
    isRed = false;
  }
  // BUG(Matthew): There actually *is* a 3rd case in the alliance values.
  // It might make sense to explicitly handle that, rather than just always
  // falling back on "It's not the red alliance, so it must be blue", without
  // any indication that something is funky.

  // Reset the intensity before starting.
  currentIntensityPercent = 0;

  // OK, set the lights to initial intensity/color.
  if (isRed) {
    m_lighting->SetAllToColor(red * currentIntensityPercent, 0, 0);
  } else {
    m_lighting->SetAllToColor(0, 0, blue * currentIntensityPercent);
  }
}

// Called repeatedly when this Command is scheduled to run
void BreathingAllianceLights::Execute() {
  if (currentIntensityPercent >= maxIntensityPercent) {
    breathingIn = false;
    increment = -0.01;
    currentIntensityPercent = maxIntensityPercent;
  }
  if (currentIntensityPercent <= 0) {
    breathingIn = true;
    increment = 0.01;
    currentIntensityPercent = 0;
  }

  if (isRed) {
    m_lighting->SetAllToColor(red * currentIntensityPercent, 0, 0);
  } else {
    m_lighting->SetAllToColor(0, 0, blue * currentIntensityPercent);
  }
  currentIntensityPercent += increment;
}

// Called once the command ends or is interrupted.
void BreathingAllianceLights::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}
