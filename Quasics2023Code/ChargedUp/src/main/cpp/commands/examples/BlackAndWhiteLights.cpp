// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/examples/BlackAndWhiteLights.h"

BlackAndWhiteLights::BlackAndWhiteLights(Lighting* lighting) {
  m_lighting = lighting;
  AddRequirements(m_lighting);
}

frc::AddressableLED::LEDData BlackAndWhiteLights::colorFunction(int position) {
  if (position % 2 == 0) {
    return Lighting::WHITE;
  } else {
    return Lighting::BLACK;
  }
}

// Called when the command is initially scheduled.
void BlackAndWhiteLights::Initialize() {
  m_lighting->SetLightColors(colorFunction);
}

// Called repeatedly when this Command is scheduled to run
void BlackAndWhiteLights::Execute() {
  m_lighting->SetLightColors(colorFunction);
}

// Called once the command ends or is interrupted.
void BlackAndWhiteLights::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool BlackAndWhiteLights::IsFinished() {
  return false;
}
