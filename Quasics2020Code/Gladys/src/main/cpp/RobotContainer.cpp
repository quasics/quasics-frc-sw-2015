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
#include "commands/IntakeBallsReverseCommand.h"
#include "commands/LoadFromWindowCommand.h"
#include "commands/MoveForTimeCommand.h"
#include "commands/ShootBallsCommand.h"
#include "commands/ShootBallsReverseCommand.h"
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
#include "commands/DriveADistance.h"
#include "commands/PointTurnToAnAngleCommand.h"
#include "commands/IntakeBallsFromFloorCommand.h"

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

void RobotContainer::ConfigureButtonBindings() {
  std::cout << "Beginning button binding configuration" << std::endl;

  //
  // Configuring buttons on the driver's controller.
  //
  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::LeftShoulder)
      .WhenPressed(enableTurboMode)
      .WhenReleased(disableTurboMode);

  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::RightShoulder)
      .WhenPressed(frontIsForward);

  // frc2::JoystickButton(&driverJoystick,
  //                      OIConstants::LogitechGamePad::LeftShoulder)
  //     .WhileHeld(IntakeBallsCommand(&intake));

  //
  // Configuring buttons on the operator's controller.
  //
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperLeft))
      .WhenPressed(TurnControlPanel4TimesCommand(&commandPanel));

  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kA))
      .WhenPressed(frc2::PrintCommand("Button 'A' on XBox was pressed"));

  // Exhaust
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kY))
      .WhileHeld(LoadFromWindowCommand(&exhaust));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kB))
      .WhileHeld(DeliverToLowGoalCommand(&exhaust));
  //

  // Intake
  // frc2::JoystickButton(
  //&operatorController,
  // int(frc::XboxController::Button::kA)
  //).WhileHeld(IntakeBallsCommand(&intake));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kA))
      .WhileHeld(IntakeBallsFromFloorCommand(&intake,&exhaust));
  //

  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kStart))
      .WhileHeld(SpinTheWheelCommand(&commandPanel, true));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperLeft))
      .WhileHeld(ClimberUpCommand(&climber));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperRight))
      .WhileHeld(ClimberDownCommand(&climber));

  // frc2::JoystickButton(&operatorController,
  // int(frc::XboxController::Button::kBumperRight))
  //.WhileHeld(TurnControlPanelToTargetColorCommand(&commandPanel));

  // Going to map the following buttons:
  //    * left bumper --> push the ball up (TODO) fu=ind other binding
  //    * B button    --> run the exhaust to push out a ball
  //    * X button    --> intake balls
  //    * back button --> ????

  std::cout << "Completed button binding configuration" << std::endl;
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

  frc::SmartDashboard::PutData("Turn the Camera Forward",
                               new TurnCameraForward(&cameraStand));

  frc::SmartDashboard::PutData("Turn the Camera Backwards",
                               new TurnCameraBackward(&cameraStand));

  frc::SmartDashboard::PutData("Toggle Camera Direction",
                               new SwitchCameraDirection(&cameraStand));
          
  frc::SmartDashboard::PutData("Move Distance (36 inches)",
                               new DriveADistance(&drivebase, 36.00, 0.25));

  frc::SmartDashboard::PutData("Turn 90 Degrees to the Right",
                               new PointTurnToAnAngleCommand(&drivebase, true, 90.0));
  
  frc::SmartDashboard::PutData("Turn 90 Degrees to the Left",
                               new PointTurnToAnAngleCommand(&drivebase, false, 90.0));
  
  frc::SmartDashboard::PutData("Turn 45 Degrees to the Right",
                               new PointTurnToAnAngleCommand(&drivebase, true, 45.0));
  
  frc::SmartDashboard::PutData("Turn 45 Degrees to the Left",
                               new PointTurnToAnAngleCommand(&drivebase, false, 45.0));

}
