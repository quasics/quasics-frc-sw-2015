// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "TrajectoryGenerator.h"
#include "commands/ArcadeDrive.h"
#include "commands/Autos.h"
#include "commands/MoveClimbers.h"
#include "commands/PivotIntake.h"
#include "commands/RunIntake.h"
#include "commands/RunShooter.h"
#include "commands/TankDrive.h"
#include "subsystems/RealDrivebase.h"
#include "subsystems/SimulatedDrivebase.h"
#include "subsystems/XRPDrivebase.h"

constexpr bool USE_XRP_UNDER_SIMULATION = false;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  allocateDriveBase();
  // setUpTankDrive();
  setUpArcadeDrive();
  AddTestButtonsOnSmartDashboard();
  // Configure the button bindings
  // ConfigureBindings();
}

/*void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
} */

void RobotContainer::setUpTankDrive() {
  // Figure out the joystick axes to be used to control driving.  (This assumes
  // that we're using the keyboard controls to simulate a joystick under the
  // simulator, or else a Logitech controller for a real robot.)
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    // On the keyboard:
    // * axis 0 is the "A" (negative)/"D" (positive) keys
    // * axis 1 is the "W" (negative)/"S" (positive) keys
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
  } else {
    leftDriveJoystickAxis = OperatorConstants::LogitechGamePad::LeftYAxis;
    rightDriveJoystickAxis = OperatorConstants::LogitechGamePad::RightYAxis;
  }

  // Create the TankDrive command (reading from the controller's joysticks), and
  // set it as the default command for the drive base.
  TankDrive::PercentSupplier leftSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();
    double joystickPercentage =
        m_driverController.GetRawAxis(leftDriveJoystickAxis) * -1;
    double joystickAfterScaling =
        m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
    return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
  };
  TankDrive::PercentSupplier rightSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();
    double joystickPercentage =
        m_driverController.GetRawAxis(rightDriveJoystickAxis) * -1;
    double joystickAfterScaling =
        m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
    return m_rightSlewRateLimiter.Calculate(joystickAfterScaling);
  };
  TankDrive tankDrive(*m_drivebase, leftSupplier, rightSupplier);
  // m_drivebase->SetDefaultCommand(std::move(tankDrive));
}

void RobotContainer::setUpArcadeDrive() {
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
  } else {
    leftDriveJoystickAxis = OperatorConstants::LogitechGamePad::LeftYAxis;
    rightDriveJoystickAxis = OperatorConstants::LogitechGamePad::RightXAxis;
  }

  ArcadeDrive::PercentSupplier forwardSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();
    double joystickPercentage =
        m_driverController.GetRawAxis(leftDriveJoystickAxis) * -1;
    double joystickAfterScaling =
        m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
    return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
  };
  ArcadeDrive::PercentSupplier rotationSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();
    double joystickPercentage =
        m_driverController.GetRawAxis(rightDriveJoystickAxis) * -1;
    return m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
  };
  ArcadeDrive arcadeDrive(*m_drivebase, forwardSupplier, rotationSupplier);
  m_drivebase->SetDefaultCommand(std::move(arcadeDrive));
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverController.GetRawButton(
      OperatorConstants::LogitechGamePad::RightShoulder);
  const bool isTurtle = m_driverController.GetRawButton(
      OperatorConstants::LogitechGamePad::LeftShoulder);

  if (isTurbo) {
    return RobotSpeedScaling::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotSpeedScaling::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotSpeedScaling::NORMAL_MODE_SPEED_SCALING;
  }
}

void RobotContainer::allocateDriveBase() {
  if (frc::RobotBase::IsReal()) {
    // OK, we're running on a "big bot".
    m_drivebase.reset(new RealDrivebase);
  } else {
    m_drivebase.reset(new SimulatedDrivebase);

    // // OK, we're running under simulation.  However, this could either mean
    // that
    // // we're talking to an XRP "little bot", or doing *pure* (GUI-based)
    // // simulation on a PC of some sort.
    // if (USE_XRP_UNDER_SIMULATION) {
    //   m_drivebase.reset(new XRPDrivebase);
    // } else {
    //   m_drivebase.reset(new SimulatedDrivebase);
    // }
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(m_drivebase.get());
}

void RobotContainer::AddTestButtonsOnSmartDashboard() {
  // This is needed because we cannot just input a command ptr onto the FRC
  // Smart Dashboard bc it will be deleted and some values that it had would be
  // still needed. So one thing sais that it needs it, but there is no real data
  // behind it.  This allows us to make this data storage more permanent.
  frc::SmartDashboard::PutData(
      "reset encoders",
      new frc2::InstantCommand([this]() { m_drivebase->ResetEncoders(); }));
  frc::SmartDashboard::PutData("reset odometry",
                               new frc2::InstantCommand([this]() {
                                 m_drivebase->resetOdometry(frc::Pose2d());
                               }));

  retainedCommands.push_back(
      GetCommandForTrajectory("test.wpilib.json", m_drivebase.get(), false));
  frc::SmartDashboard::PutData("test path", retainedCommands.rbegin()->get());

  retainedCommands.push_back(GetCommandForTrajectory("curvetest.wpilib.json",
                                                     m_drivebase.get(), false));
  frc::SmartDashboard::PutData("curve test path",
                               retainedCommands.rbegin()->get());
}

void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&m_driverController, logitechButtonId)
      .WhileTrue(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command* command) {
  frc2::JoystickButton(&m_operatorController, buttonId).WhileTrue(command);
}

void RobotContainer::ConfigureDriverControllerButtonBindings() {
  static MoveClimbers extendClimbers(&m_climber, true);
  static MoveClimbers retractClimbers(&m_climber, false);
  static RunIntake intakeNote(&m_intakeRoller, 0.5, true);
  static RunIntake dropNote(&m_intakeRoller, 0.5, false);

  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::YButton,
                                   &extendClimbers);
  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::AButton,
                                   &retractClimbers);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::LeftShoulder, &dropNote);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::RightShoulder, &intakeNote);
}

void RobotContainer::ConfigureOperatorControllerButtonBindings() {
  static PivotIntake extendIntake(&m_intakeDeployment, 0.5, true);
  static PivotIntake retractIntake(&m_intakeDeployment, 0.5, false);
  static RunShooter shootNote(&m_shooter, 0.5, true);
  static RunShooter retractNote(&m_shooter, -0.5, true);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &extendIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &retractIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     &shootNote);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kX,
                                     &retractNote);
}