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

// Software dead-band handling on readings from the driver's control.
inline double applyDeadBandAdjustment(double stickValue) {
  if (std::abs(stickValue) < OIConstants::kDriveControllerDeadBandSize) {
    return 0;
  }
  return stickValue;
}

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_driveBase.SetDefaultCommand(TankDriveCommand(
      &m_driveBase,
      [this] {
        return applyDeadBandAdjustment(
            m_driverController.GetY(frc::GenericHID::kLeftHand));
      },
      [this] {
        return applyDeadBandAdjustment(
            m_driverController.GetX(frc::GenericHID::kRightHand));
      }));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Trivial example, just for demo purposes.
  frc2::JoystickButton(&m_driverController, int(frc::XboxController::Button::kA))
      .WhenPressed(new frc2::PrintCommand("Button 'A' was pressed"));

  // While holding the shoulder button, enable turbo mode
  frc2::JoystickButton(&m_driverController, int(frc::XboxController::Button::kBumperRight))
      .WhenPressed(&m_enableTurbo)
      .WhenReleased(&m_disableTurbo);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
