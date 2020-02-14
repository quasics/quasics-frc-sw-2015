/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinTheWheel.h"

SpinTheWheel::SpinTheWheel(CommandPanel*controlPanel, bool forward):m_controlPanel(controlPanel),f(forward)  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void SpinTheWheel::Initialize() {
  m_controlPanel->TurnWheelMotorOn(f);
}

// Called repeatedly when this Command is scheduled to run
void SpinTheWheel::Execute() {}

// Called once the command ends or is interrupted.
void SpinTheWheel::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool SpinTheWheel::IsFinished() { 
  
  return false;
   }
