/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinTheWheelCommand.h"

SpinTheWheelCommand::SpinTheWheelCommand(CommandPanel* controlPanel,
                                         bool forward)
    : m_controlPanel(controlPanel), f(forward) {
  AddRequirements(m_controlPanel);
}

// Called when the command is initially scheduled.
void SpinTheWheelCommand::Initialize() {
  m_controlPanel->TurnWheelMotorOn(f);
}


// Called once the command ends or is interrupted.
void SpinTheWheelCommand::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// TODO(RJ): Remove this unneeded method (from .cpp and header).
// Returns true when the command should end.
bool SpinTheWheelCommand::IsFinished() {
  return false;
}
