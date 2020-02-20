/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanel4TimesCommand.h"

TurnControlPanel4TimesCommand::TurnControlPanel4TimesCommand(
    CommandPanel* controlPanel)
    : m_controlPanel(controlPanel) {
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void TurnControlPanel4TimesCommand::Initialize() {
  std::cout << "Initializing 'Turn 4 times'" << std::endl;
  initColor = prevColor = m_controlPanel->getCurrentColor();
  std::cout << "Initial color is " << m_controlPanel->getColorName(initColor)
            << std::endl;
  // TODO(RJ): What if you can't get the initial color?  (In other words, what
  // if the sensor hands back an unknown/unexpected value?  How should this case
  // be handled?)

  m_controlPanel->TurnWheelMotorOn(true);
}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanel4TimesCommand::Execute() {
  // TODO(RJ): Remove this empty function (both here and in header).
  ;
}

// Called once the command ends or is interrupted.
void TurnControlPanel4TimesCommand::End(bool interrupted) {
  std::cout << "Ending 'Turn 4 times'" << std::endl;
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool TurnControlPanel4TimesCommand::IsFinished() {
  currColor = m_controlPanel->getCurrentColor();
  if (prevColor != currColor) {
    std::cout << "New detected color is "
              << m_controlPanel->getColorName(currColor) << std::endl;
    if (initColor == currColor) {
      std::cout << "Bump the count!" << std::endl;
      counter++;
      if (counter > 7) {
        return true;
      }
    }
  }
  prevColor = currColor;
  return false;
}
