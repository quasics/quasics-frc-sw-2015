// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/examples/SplitLightingExample.h"

SplitLightingExample::SplitLightingExample(
    Lighting* lighting, frc::AddressableLED::LEDData frontColor,
    frc::AddressableLED::LEDData rearColor)
    : m_lighting(lighting),
      m_colorFunction{
          // We need to "capture" the values passed in for the colors
          [frontColor, rearColor]
          // And then we can define the rest of the lambda function
          (int pos) {
            if (Lighting::IsFrontSideLED(pos))
              return frontColor;
            else
              return rearColor;
          }} {
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void SplitLightingExample::Initialize() {
  m_lighting->SetLightColors(m_colorFunction);
}

// Called repeatedly when this Command is scheduled to run
void SplitLightingExample::Execute() {
  m_lighting->SetLightColors(m_colorFunction);
}

// Called once the command ends or is interrupted.
void SplitLightingExample::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool SplitLightingExample::IsFinished() {
  return false;
}
