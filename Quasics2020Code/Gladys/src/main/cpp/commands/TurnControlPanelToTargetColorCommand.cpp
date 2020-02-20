/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanelToTargetColorCommand.h"

#include <frc/DriverStation.h>

// TODO(RJ): (Bug) Remove this global variable.  (Make it local to the function
// where it's used.)
bool noData = false;

TurnControlPanelToTargetColorCommand::TurnControlPanelToTargetColorCommand(
    CommandPanel* controlPanel)
    : m_controlPanel(controlPanel), m_aimColor(CommandPanel::UNKNOWN) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TurnControlPanelToTargetColorCommand::Initialize() {
  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if (gameData.length() > 0) {
    noData = false;
    switch (gameData[0]) {
      case 'B':
        // Blue case code
        m_aimColor = CommandPanel::BLUE;
        break;
      case 'G':
        // Green case code
        m_aimColor = CommandPanel::GREEN;
        break;
      case 'R':
        // Red case code
        m_aimColor = CommandPanel::RED;
        break;
      case 'Y':
        // Yellow case code
        m_aimColor = CommandPanel::YELLOW;
        break;
      default:
        // This is corrupt data
        m_aimColor = CommandPanel::UNKNOWN;
        break;
    }
    m_controlPanel->TurnWheelMotorOn(true);
  } else {
    // Code for no data received yet
    noData = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void TurnControlPanelToTargetColorCommand::Execute() {
}

// Called once the command ends or is interrupted.
void TurnControlPanelToTargetColorCommand::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool TurnControlPanelToTargetColorCommand::IsFinished() {
  if (m_aimColor == m_controlPanel->getCurrentColor() || !noData) {
    return true;
  }
  return false;
}
