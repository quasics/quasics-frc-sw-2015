// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <filesystem>

#include "../../../../Common2021/TeleopArcadeDrive.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "Constants.h"
#include "commands/DriveForward.h"

#undef DRIVE_ARCADE_STYLE
#define TURN_TO_TARGET_AUTO
#define USE_GAMESIR_CONTROLLER

// Unfortunately, the Logitech controllers can't be used with the Mac OS,
// which is what Matt has handy.  So here's a convenient hack to encapsulate the
// idea of what type of device we're using.
inline bool usingLogitechController() {
#ifdef USE_GAMESIR_CONTROLLER
  return false;
#else
  return true;
#endif
}

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Note: when running under simulator, current path is that of the
  // project (.../Experimental/RomiToy2021).
  m_helper.InstallSliders();

  ConfigureDrivingCommand();

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::EnableTankDrive() {
  m_drive.SetDefaultCommand(TeleopTankDrive(
      &m_drive,
      [this] {
        const int leftJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -m_controller.GetRawAxis(leftJoystickAxis);
      },
      [this] {
        const int rightJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::RightYAxis
                : int(JoystickDefinitions::GameSirPro::RightVertical);
        return -m_controller.GetRawAxis(rightJoystickAxis);
      },
      [this] {
        static bool switchDriveEnabled = false;
        static bool switchDriveButtonPressedLastTime = false;
        const int switchDriveButton =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::StartButton
                : int(JoystickDefinitions::GameSirPro::S);
        bool buttonPressed = m_controller.GetRawButton(switchDriveButton);
        if (buttonPressed && !switchDriveButtonPressedLastTime) {
          switchDriveEnabled = !switchDriveEnabled;
        }
        switchDriveButtonPressedLastTime = buttonPressed;
        return switchDriveEnabled;
      }));
}

void RobotContainer::EnableArcadeDrive() {
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
}

void RobotContainer::ConfigureDrivingCommand() {
#ifdef DRIVE_ARCADE_STYLE
  EnableArcadeDrive();
#else
  EnableTankDrive();
#endif
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
#ifdef TURN_TO_TARGET_AUTO
  return &m_turnToTargetCommand;
#else
  // An example command will be run in autonomous
  return &m_autonomousCommand;
#endif
}
