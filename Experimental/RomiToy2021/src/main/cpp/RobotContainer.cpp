// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "../../../../Common2021/TeleopArcadeDrive.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "Constants.h"
#include "commands/DriveForward.h"

#undef DRIVE_ARCADE_STYLE

// Unfortunately, the Logitech controllers can't be used with the Mac OS,
// which is what Matt has handy.  So here's a convenient hack to use the
// "game specific data" field to figure out what kind of device we should
// read from.
inline bool usingLogitechController() {
  std::string msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  return (msg != "MattsMac");
}

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

#ifdef DRIVE_ARCADE_STYLE
  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive, [this] {
        const int joystickVerticalAxis = usingLogitechController()
                                        ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                                        : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -m_controller.GetRawAxis(joystickVerticalAxis);
      },
      [this] {
        const int joystickHorizontalalAxis = usingLogitechController()
                                        ? JoystickDefinitions::LogitechGamePad::LeftXAxis
                                        : int(JoystickDefinitions::GameSirPro::LeftHorizontal);
        return m_controller.GetRawAxis(joystickHorizontalalAxis);
      }));
#else
  m_drive.SetDefaultCommand(TeleopTankDrive(
      &m_drive,
      [this] {
        const int leftJoystickAxis = usingLogitechController()
                                        ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                                        : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -m_controller.GetRawAxis(leftJoystickAxis);
      },
      [this] {
        const int rightJoystickAxis = usingLogitechController()
                                        ? JoystickDefinitions::LogitechGamePad::RightYAxis
                                        : int(JoystickDefinitions::GameSirPro::RightVertical);
        return -m_controller.GetRawAxis(rightJoystickAxis);
      }));
#endif

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  DriveForward forward(
      &m_drive,
      .5,                    // Power setting
      [] { return false; },  // Stop condition (move until interrupted)
      DriveForward::SensorMode::Gyro,  // Sensor mode
      false                            // Noisy?
  );
  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::G))
      .WhenPressed(frc2::PrintCommand("Button 'G' on controller was pressed"));
  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::A))
      .WhenHeld(forward);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
