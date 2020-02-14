/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Turn4Times.h"

// TODO(RJ): Turn these global variables into local variables, or members of
// this class, as appropriate.

Turn4Times::Turn4Times(CommandPanel*controlPanel):m_controlPanel(controlPanel) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void Turn4Times::Initialize() {
  std::cout << "Initializing 'Turn 4 times'" << std::endl;
  initColor = prevColor = m_controlPanel->getCurrentColor();
  m_controlPanel->TurnWheelMotorOn(true); 
}

// Called repeatedly when this Command is scheduled to run
void Turn4Times::Execute() {
// TODO(RJ): Remove this empty function (both here and in header).
  ;
}

// Called once the command ends or is interrupted.
void Turn4Times::End(bool interrupted) {
  std::cout << "Ending 'Turn 4 times'" << std::endl;
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool Turn4Times::IsFinished() {
  currColor = m_controlPanel->getCurrentColor();
  if (prevColor != currColor) {
    std::cout << "New color is " << m_controlPanel->getColorName(initColor) << std::endl;
    if(initColor == currColor){
      std::cout << "Bump the count!" << std::endl;
      counter++;
      if(counter>7){
        return true;
      }
    }
  }
  return false;
}
