/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnControlPanelToTargetColorCommand.h"

#include <frc/DriverStation.h>

TurnControlPanelToTargetColorCommand::TurnControlPanelToTargetColorCommand(
    ControlPanel* controlPanel)
    : m_controlPanel(controlPanel), m_aimColor(ControlPanel::UNKNOWN) {
  // Use AddRequirements() here to declare subsystem dependencies.
  /// TODO(RJ): (BUG) Add the control panel as a required subsystem, per the
  // above comment!
}

//// TODO(RJ): (BUG) This code will turn the control panel until the color
/// sensor
/// on the robot detects the color specified by the FMS.  However, this isn't
/// what's needed, since the color specified by the FMS is the color that the
/// *field* sensor needs to detect, and the robot will be looking at a different
/// point (color) on the wheel.
///
/// As we discussed earlier in build season, you need to convert the color
/// specified by the FMS to the color that we need to detect from the robot's
/// sensor position (probably about 90 degrees off from the field sensor).

// Called when the command is initially scheduled.
void TurnControlPanelToTargetColorCommand::Initialize() {
  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if (gameData.length() > 0) {
    switch (gameData[0]) {
      case 'B':
        // Blue case code
        m_aimColor = ControlPanel::BLUE;
        break;
      case 'G':
        // Green case code
        m_aimColor = ControlPanel::GREEN;
        break;
      case 'R':
        // Red case code
        m_aimColor = ControlPanel::RED;
        break;
      case 'Y':
        // Yellow case code
        m_aimColor = ControlPanel::YELLOW;
        break;
      default:
        // This is corrupt data
        m_aimColor = ControlPanel::UNKNOWN;
        break;
    }
    m_controlPanel->TurnWheelMotorOn(true);
  } else {
    // Code for no data received yet
    m_aimColor = ControlPanel::NO_DATA;
  }
}

// Called once the command ends or is interrupted.
void TurnControlPanelToTargetColorCommand::End(bool interrupted) {
  m_controlPanel->TurnWheelMotorOff();
}

// Returns true when the command should end.
bool TurnControlPanelToTargetColorCommand::IsFinished() {
  if (m_aimColor == m_controlPanel->getCurrentColor() ||
      m_aimColor == ControlPanel::NO_DATA) {
    return true;
  }
  return false;
}
