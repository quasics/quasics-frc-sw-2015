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
  std::cout << "Initializing color strip data." << std::endl;
  lights->SetStripColor(red, green, blue);
}
