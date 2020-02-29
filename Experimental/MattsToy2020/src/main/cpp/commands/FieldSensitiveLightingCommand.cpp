/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FieldSensitiveLightingCommand.h"

#include <frc/DriverStation.h>

// Called repeatedly when this Command is scheduled to run
void FieldSensitiveLightingCommand::Execute() {
  const auto colorData = frc::DriverStation::GetInstance().GetAlliance();
  switch (colorData) {
    case frc::DriverStation::kRed:
      lighting->SetStripToSingleColor(LightingSubsystem::Color::red);
      break;
    case frc::DriverStation::kBlue:
      lighting->SetStripToSingleColor(LightingSubsystem::Color::blue);
      break;
    case frc::DriverStation::kInvalid:
    default:
      lighting->SetStripToSingleColor(LightingSubsystem::Color::green);
      ;
      break;
  }
}

// Called once the command ends or is interrupted.
void FieldSensitiveLightingCommand::End(bool interrupted) {
  lighting->SetStripToSingleColor(LightingSubsystem::Color::green);
}
