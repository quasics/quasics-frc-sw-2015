/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Set names of commands
  m_instantCommand1.SetName("Instant Command 1");
  m_instantCommand2.SetName("Instant Command 2");
  m_waitCommand.SetName("Wait 5 Seconds Command");

  // Set the scheduler to log Shuffleboard events for command initialize,
  // interrupt, finish
  frc2::CommandScheduler::GetInstance().OnCommandInitialize(
      [](const frc2::Command& command) {
        frc::Shuffleboard::AddEventMarker(
            "Command Initialized", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });
  frc2::CommandScheduler::GetInstance().OnCommandInterrupt(
      [](const frc2::Command& command) {
        frc::Shuffleboard::AddEventMarker(
            "Command Interrupted", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });
  frc2::CommandScheduler::GetInstance().OnCommandFinish(
      [](const frc2::Command& command) {
        frc::Shuffleboard::AddEventMarker(
            "Command Finished", command.GetName(),
            frc::ShuffleboardEventImportance::kNormal);
      });

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // Run instant command 1 when the 'A' button is pressed
  frc2::JoystickButton(&m_driverController, 0).WhenPressed(&m_instantCommand1);
  // Run instant command 2 when the 'X' button is pressed
  frc2::JoystickButton(&m_driverController, 3).WhenPressed(&m_instantCommand2);
  // Run instant command 3 when the 'Y' button is held; release early to
  // interrupt
  frc2::JoystickButton(&m_driverController, 4).WhenHeld(&m_waitCommand);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // no auto
  return nullptr;
}
