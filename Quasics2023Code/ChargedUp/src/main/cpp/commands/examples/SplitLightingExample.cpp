// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/examples/SplitLightingExample.h"

SplitLightingExample::SplitLightingExample(
    Lighting* lighting, frc::AddressableLED::LEDData frontColor,
    frc::AddressableLED::LEDData rearColor)
    : m_lighting(lighting), m_frontColor(frontColor), m_rearColor(rearColor) {
  AddRequirements(m_lighting);
}

frc::AddressableLED::LEDData SplitLightingExample::ColorFunction(int pos) {
  if (Lighting::IsFrontSideLED(pos))
    return m_frontColor;
  else
    return m_rearColor;
}

void SplitLightingExample::UpdateColors() {
  m_lighting->SetLightColors(std::bind(
      // This is the member function we want to have invoked
      &SplitLightingExample::ColorFunction,
      // It (invisibly) has "this" passed as its first parameter.
      this,
      // There's another paramter that will be provided when the function is
      // *used*, so we provide a "placeholder" for the bind() function.
      std::placeholders::_1));
}

// Called when the command is initially scheduled.
void SplitLightingExample::Initialize() {
  UpdateColors();
}

// Called repeatedly when this Command is scheduled to run
void SplitLightingExample::Execute() {
  UpdateColors();
}

// Called once the command ends or is interrupted.
void SplitLightingExample::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool SplitLightingExample::IsFinished() {
  return false;
}
