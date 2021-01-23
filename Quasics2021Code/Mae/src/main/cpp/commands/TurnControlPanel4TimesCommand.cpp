/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanel4TimesCommand.h"

TurnControlPanel4TimesCommand::TurnControlPanel4TimesCommand(
    ControlPanel* controlPanel)
    : m_controlPanel(controlPanel) {
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void TurnControlPanel4TimesCommand::Initialize() {
  counter = 0;
  initColor = prevColor = m_controlPanel->getCurrentColor();
  while (initColor == ControlPanel::UNKNOWN) {
    initColor = prevColor = m_controlPanel->getCurrentColor();
  }

  m_controlPanel->TurnWheelMotorOn(true);
}

// Called once the command ends or is interrupted.
void TurnControlPanel4TimesCommand::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool TurnControlPanel4TimesCommand::IsFinished() {
  currColor = m_controlPanel->getCurrentColor();
  if (prevColor != currColor) {
    std::cout << "New detected color is "
              << m_controlPanel->getColorName(currColor) << std::endl;
    if (initColor == currColor) {
      counter++;
      std::cout << "Bumped the # of occurrences of target color to "
                << counter
                << std::endl;
      if (counter > 7) {
        return true;
      }
    }
  }
  prevColor = m_controlPanel->getCurrentColor();
  return false;
}
