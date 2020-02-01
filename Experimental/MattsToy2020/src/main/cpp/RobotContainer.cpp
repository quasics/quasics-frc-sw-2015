/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "commands/TankDriveCommand.h"

// Function to perform software dead-band handling on readings from the driver's
// control.
inline double applyDeadBandAdjustment(double stickValue) {
  if (std::abs(stickValue) < OIConstants::kDriveControllerDeadBandSize) {
    return 0;
  }
  return stickValue;
}

// Initialize all of your commands and subsystems here.
RobotContainer::RobotContainer() {
  m_driveBase.SetDefaultCommand(TankDriveCommand(
      &m_driveBase,
      [this] {
        double stickValue = m_logitechController.GetRawAxis(OIConstants::LogitechGamePad_LeftYAxis);
        return applyDeadBandAdjustment(stickValue);
      },
      [this] {
        double stickValue = m_logitechController.GetRawAxis(OIConstants::LogitechGamePad_RightYAxis);
        return applyDeadBandAdjustment(stickValue);
      }));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Trivial example, just for demo purposes.
  frc2::JoystickButton(&m_xboxController, int(frc::XboxController::Button::kA))
      .WhenPressed(new frc2::PrintCommand("Button 'A' on XBox was pressed"));

  // While holding the shoulder button, enable turbo mode
  frc2::JoystickButton(&m_logitechController, OIConstants::LogitechGamePad_LeftShoulder)
      .WhenPressed(&m_enableTurbo)
      .WhenReleased(&m_disableTurbo);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
