/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/BasicLightingControlCommand.h"
#include "subsystems/Lights.h"

BasicLightingControlCommand::BasicLightingControlCommand(Lights*lights,Color control):
  lights(lights),
  control(control)
  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(lights);
}

// Called when the command is initially scheduled.
void BasicLightingControlCommand::Initialize() {
  switch(control) {
    case Color::Red:
      lights->SetLightsColor(255, 0, 0);
      break;
    case Color::Blue:
      lights->SetLightsColor(0, 0, 255);
      break;
    case Color::Green:
      lights->SetLightsColor(0, 255, 0);
      break;
  }
}

// Called once the command ends or is interrupted.
void BasicLightingControlCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool BasicLightingControlCommand::IsFinished() { return true; }
