// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ColorLights.h"

ColorLights::ColorLights(Lights* lights, int red, int green, int blue)
: lights(lights), red(red), green(green), blue(blue) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(lights);
}

// Called when the command is initially scheduled.
void ColorLights::Initialize() {
  lights->SetStripColor(red, green, blue);
}

// Called repeatedly when this Command is scheduled to run
void ColorLights::Execute() {}

// Called once the command ends or is interrupted.
void ColorLights::End(bool interrupted) {}

// Returns true when the command should end.
bool ColorLights::IsFinished() {
  return false;
}
