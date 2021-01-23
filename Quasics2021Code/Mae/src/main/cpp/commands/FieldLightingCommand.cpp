/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FieldLightingCommand.h"
#include <frc/DriverStation.h>

FieldLightingCommand::FieldLightingCommand(Lights*lights):
lights(lights) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(lights);
}

// Called when the command is initially scheduled.
void FieldLightingCommand::Initialize() {
  lights->TurnOff();
}

// Called repeatedly when this Command is scheduled to run
void FieldLightingCommand::Execute() {
  const auto colorData = frc::DriverStation::GetInstance().GetAlliance();
  switch(colorData){
    case frc::DriverStation::kRed:
    lights->SetLightsColor(255, 0, 0);
    break;
    case frc::DriverStation::kBlue:
    lights->SetLightsColor(0, 0, 255);
    break;
    case frc::DriverStation::kInvalid:
    default:
    lights->SetLightsColor(0, 255, 0);
    break;
  }
}

// Called once the command ends or is interrupted.
void FieldLightingCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool FieldLightingCommand::IsFinished() { return false; }
