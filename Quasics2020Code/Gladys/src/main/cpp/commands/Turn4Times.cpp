/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Turn4Times.h"
int counter=0;
CommandPanel::Color initColor=CommandPanel::UNKNOWN, prevColor=CommandPanel::UNKNOWN, currColor=CommandPanel::UNKNOWN;
Turn4Times::Turn4Times(CommandPanel*controlPanel):m_controlPanel(controlPanel) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void Turn4Times::Initialize() {
  initColor = prevColor = m_controlPanel->getCurrentColor();
  m_controlPanel->TurnWheelMotorOn(); 
}

// Called repeatedly when this Command is scheduled to run
void Turn4Times::Execute() {
  ;
}

// Called once the command ends or is interrupted.
void Turn4Times::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool Turn4Times::IsFinished() {
  currColor = m_controlPanel->getCurrentColor();
  if(prevColor!=currColor && initColor == currColor){
    counter++;
    if(counter>7){
      return true;
    }
  }
  return false;
}
