// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>

#include <memory>

#include "commands/AutoModeBallDelivery.h"
#include "commands/MoveForTimeCommand.h"
#include "subsystems/CameraStand.h"
#include "subsystems/Climber.h"
#include "subsystems/ControlPanel.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Exhaust.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void ResetDriveEncoders() { drivebase.ResetEncoderPositions(); }

  // The robot's subsystems and commands are defined here...
 private:
  Drivebase drivebase;
  // "Instant" commands, to quietly modify the drive base's state
  frc2::InstantCommand enableTurboMode{[this] { drivebase.EnableTurboMode(); },
                                       {}};
  frc2::InstantCommand disableTurboMode{
      [this] { drivebase.DisableTurboMode(); }, {}};
  frc2::InstantCommand frontIsForward{[this] { drivebase.SwitchFace(); }, {}};

  // Other subsystems
  Intake intake;
  Exhaust exhaust;
  Climber climber;
  ControlPanel commandPanel;
  CameraStand cameraStand;
  Lights lights;

  // Controllers
  frc::Joystick driverJoystick{0};
  frc::XboxController operatorController{1};

  /// @todo (Scott) Replace this with a real command for autonomous mode.  (Or
  // more than one.)
  // MoveForTimeCommand m_trivialAutonmousCommand{drivebase.get(), 2, -.40};
  AutoModeBallDelivery m_autonomousCommand;

 private:
  // Private utility/configuration functions are defined here.
  void ConfigureSmartDashboard();
  void ConfigureButtonBindings();
  void ConfigureDriverButtonBindings();
  void ConfigureOperatorButtonBindings();

  void RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button buttonId,
                                          frc2::Command* command);

  void RunCommandWhenOperatorButtonIsPressed(
      frc::XboxController::Button buttonId, frc2::Command* command);
};
