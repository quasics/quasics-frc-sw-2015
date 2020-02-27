/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>

#include "Constants.h"
#include "commands/ClimberDownCommand.h"
#include "commands/ClimberUpCommand.h"
#include "commands/DeliverToLowGoalCommand.h"
#include "commands/DriveADistanceCommand.h"
#include "commands/IntakeBallsFromFloorCommand.h"
#include "commands/LoadFromWindowCommand.h"
#include "commands/MoveForTimeCommand.h"
#include "commands/PointTurnToAnAngleCommand.h"
#include "commands/ShoulderControlCommand.h"
#include "commands/SpinTheWheelCommand.h"
#include "commands/SwitchCameraDirection.h"
#include "commands/TankDriveCommand.h"
#include "commands/TurnCameraBackward.h"
#include "commands/TurnCameraForward.h"
#include "commands/TurnControlPanel4TimesCommand.h"
#include "commands/TurnControlPanelToTargetColorCommand.h"
#include "subsystems/CameraStand.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"

inline double DeadBand(double stickValue) {
  if (stickValue > OIConstants::DeadBand_LowValue &&
      stickValue < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return (stickValue);
}

RobotContainer::RobotContainer() : m_autonomousCommand(&exhaust, &drivebase) {
  // Initialize all of your commands and subsystems here
  drivebase.SetDefaultCommand(TankDriveCommand(
      &drivebase,
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::RightYAxis);
        return DeadBand(stickValue);
      },
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return DeadBand(stickValue);
      }));

  intake.SetDefaultCommand(ShoulderControlCommand(&intake, [this] {
    double stickValue =
        operatorController.GetRawAxis(OIConstants::XBox::LeftYAxis);
    return DeadBand(stickValue);
  }));

  // Configure the operator interface
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
}

//
// Configuring buttons on the driver's controller.
//
void RobotContainer::ConfigureDriverButtonBindings() {
  std::cout << "Beginning button binding configuration (Driver)" << std::endl;

  namespace LogitechGamePad = OIConstants::LogitechGamePad;
  using frc2::JoystickButton;

  JoystickButton(&driverJoystick, LogitechGamePad::LeftShoulder)
      .WhenPressed(enableTurboMode)
      .WhenReleased(disableTurboMode);

  JoystickButton(&driverJoystick, LogitechGamePad::RightShoulder)
      .WhenPressed(frontIsForward);

  std::cout << "Completed button binding configuration (Driver)" << std::endl;
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    frc::XboxController::Button buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, int(buttonId)).WhileHeld(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsPressed(
    frc::XboxController::Button buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, int(buttonId)).WhenPressed(command);
}

//
// Configuring buttons on the operator's controller.
//
void RobotContainer::ConfigureOperatorButtonBindings() {
  std::cout << "Beginning button binding configuration (Operator)" << std::endl;

  typedef frc::XboxController::Button Button;  // For convenient/shorter naming

  //
  // Commands that we want to let the operator's controller trigger.
  static LoadFromWindowCommand loadFromWindow(&exhaust);
  static DeliverToLowGoalCommand deliverToGoal(&exhaust);
  static SpinTheWheelCommand spinTheWheel(&commandPanel, true);
  static ClimberUpCommand climberUp(&climber);
  static ClimberDownCommand climberDown(&climber);
  static IntakeBallsFromFloorCommand intakeFromFloor(&intake, &exhaust);
  static TurnControlPanel4TimesCommand turnPanel4Times(&commandPanel);
  static TurnControlPanelToTargetColorCommand turnPanelToColor(&commandPanel);

  //
  // Bind the commands to the buttons.

  // Exhaust
  RunCommandWhenOperatorButtonIsHeld(Button::kY, &loadFromWindow);
  RunCommandWhenOperatorButtonIsHeld(Button::kB, &deliverToGoal);

  // Intake
  RunCommandWhenOperatorButtonIsHeld(Button::kA, &intakeFromFloor);
  RunCommandWhenOperatorButtonIsHeld(Button::kBumperLeft, &climberUp);
  RunCommandWhenOperatorButtonIsHeld(Button::kBumperRight, &climberDown);

  // Control panel
  RunCommandWhenOperatorButtonIsHeld(Button::kStart, &spinTheWheel);
#if 0
  //    ***
  //    *** These are currently disabled: the buttons are already in use....
  //    ***
  RunCommandWhenOperatorButtonIsPressed(Button::kBumperLeft, &turnPanel4Times);
  RunCommandWhenOperatorButtonIsPressed(Button::kBumperRight,
                                        &turnPanelToColor);
#endif

  std::cout << "Completed button binding configuration (Operator)" << std::endl;
}

void RobotContainer::ConfigureButtonBindings() {
  ConfigureDriverButtonBindings();
  ConfigureOperatorButtonBindings();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return &m_autonomousCommand;
}

void RobotContainer::ConfigureSmartDashboard() {
  std::cout << "Beginning smart dashboard configuration" << std::endl;
  frc::SmartDashboard::PutData("Move off the line",
                               new MoveForTimeCommand(&drivebase, 3, .4));
  frc::SmartDashboard::PutData("Auto mode ball delivery",
                               new AutoModeBallDelivery(&exhaust, &drivebase));
  frc::SmartDashboard::PutData(
      "Turn Four Times", new TurnControlPanel4TimesCommand(&commandPanel));
  std::cout << "Completed smart dashboard configuration" << std::endl;

  frc::SmartDashboard::PutData(
      "Turn To Color", new TurnControlPanelToTargetColorCommand(&commandPanel));

#ifdef ENABLE_CAMERA_TEST_COMMANDS
  frc::SmartDashboard::PutData("Turn the Camera Forward",
                               new TurnCameraForward(&cameraStand));

  frc::SmartDashboard::PutData("Turn the Camera Backwards",
                               new TurnCameraBackward(&cameraStand));

  frc::SmartDashboard::PutData("Toggle Camera Direction",
                               new SwitchCameraDirection(&cameraStand));
#endif  // ENABLE_CAMERA_TEST_COMMANDS

#ifdef ENABLE_AUTO_DRIVING_TEST_COMMANDS
  frc::SmartDashboard::PutData(
      "Move Distance (36 inches)",
      new DriveADistanceCommand(&drivebase, 36.00, 0.25));

  frc::SmartDashboard::PutData(
      "Move Distance (-36 inches)",
      new DriveADistanceCommand(&drivebase, 36.00, -0.25));

  frc::SmartDashboard::PutData(
      "Turn 90 Degrees to the Right",
      new PointTurnToAnAngleCommand(&drivebase, true, 90.0));

  frc::SmartDashboard::PutData(
      "Turn 90 Degrees to the Left",
      new PointTurnToAnAngleCommand(&drivebase, false, 90.0));

  frc::SmartDashboard::PutData(
      "Turn 45 Degrees to the Right",
      new PointTurnToAnAngleCommand(&drivebase, true, 45.0));

  frc::SmartDashboard::PutData(
      "Turn 45 Degrees to the Left",
      new PointTurnToAnAngleCommand(&drivebase, false, 45.0));
#endif  // ENABLE_AUTO_DRIVING_TEST_COMMANDS
}
