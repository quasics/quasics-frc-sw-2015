/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToColor.h"

TurnToColor::TurnToColor(CommandPanel*controlPanel, CommandPanel::Color aimColor):m_controlPanel(controlPanel),m_aimColor(aimColor) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void TurnToColor::Initialize() {
  m_controlPanel->TurnWheelMotorOn();
}

// Called repeatedly when this Command is scheduled to run
void TurnToColor::Execute() {}

// Called once the command ends or is interrupted.
void TurnToColor::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool TurnToColor::IsFinished() { 
  if(m_aimColor == m_controlPanel->getCurrentColor()){
    return true;
  }
  return false; 
}
