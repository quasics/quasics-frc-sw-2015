// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/examples/SplitLightingExample.h"

SplitLightingExample::SplitLightingExample(
    Lighting* lighting, frc::AddressableLED::LEDData frontColor,
    frc::AddressableLED::LEDData rearColor)
    : m_lighting(lighting), m_frontColor(frontColor), m_rearColor(rearColor) {
  AddRequirements(m_lighting);
  SetName("SplitLightingExample");
}

// This is a "normal" member function, which means that it must be called via an
// object (e.g., "aCommand.ColorFunction(4)", or "cmdPtr->ColorFunction(6)").
// As a result, it can't be passed directly to the "SetLightsColors" function in
// the code below; we'll instead need to use "std::bind" to set things up so
// that it has a suitable wrapper that lets it be used like a "raw" function.
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
      // It (invisibly) has "this" passed as its first parameter, providing the
      // object on which the function was called.
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
