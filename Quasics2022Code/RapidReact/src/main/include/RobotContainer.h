// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "TrajectoryCommandGenerator.h"
#include "subsystems/Climber.h"
#include "subsystems/Conveyor.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/Lighting.h"
#include "subsystems/PhotonVision.h"
#include "subsystems/Shooter.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // Public methods, used by the Robot class.
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  // Commands for button bindings
  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command* command);

  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command* command);

  // "Helper" functions, used internally.
 private:
  frc2::SequentialCommandGroup* BuildShootAndMoveCommand(
      double powerShoot, units::second_t timeShoot, double powerMove,
      units::meter_t distanceToMove);

  frc2::SequentialCommandGroup* ConveyorShootingAuto();

  frc2::SequentialCommandGroup* RSM2Manual();

  frc2::SequentialCommandGroup* BSM4Manual();
  frc2::SequentialCommandGroup* Pickup1Shoot2FacingSide();
  frc2::SequentialCommandGroup* Pickup1Shoot2FacingClimbers();
  frc2::SequentialCommandGroup* Pickup1Shoot2FacingHuman();
  frc2::ParallelRaceGroup* BuildMaualDrivePickup(units::meter_t distance);

  // autoCommands

  frc2::SequentialCommandGroup* RSM1();
  frc2::SequentialCommandGroup* RSM2();
  frc2::SequentialCommandGroup* BSM3();
  frc2::SequentialCommandGroup* BSM4();

  frc2::SequentialCommandGroup* RSP1();
  frc2::SequentialCommandGroup* RSP2();
  frc2::SequentialCommandGroup* RSP3();
  frc2::SequentialCommandGroup* BSP4();
  frc2::SequentialCommandGroup* BSP5();
  frc2::SequentialCommandGroup* BSP6();

  frc2::SequentialCommandGroup* RSPS1();
  frc2::SequentialCommandGroup* BSPS2();

  frc2::SequentialCommandGroup* RPSPS1();
  frc2::SequentialCommandGroup* BPSPS2();

  frc2::SequentialCommandGroup* RALL1();
  frc2::SequentialCommandGroup* BALL2();

  // example auto command
  frc2::ParallelRaceGroup* ButtonShootingHighGoal();
  frc2::ParallelRaceGroup* ButtonShootingLowGoal();
  frc2::SequentialCommandGroup* ConveyorDelay();
  frc2::SequentialCommandGroup* ConveyorRetractionDelay();
  frc2::SequentialCommandGroup* BuildExampleAutonomousCommand();

  // helper functions for auto commands
  frc2::ParallelCommandGroup* DrivingBackToShoot(std::string Path);
  frc2::SequentialCommandGroup* DrivePickUpPositionBall(std::string Path);
  frc2::ParallelRaceGroup* DrivingAndPickingUpBall(std::string Path);
  frc2::SequentialCommandGroup* PickingUpBall();
  frc2::ParallelCommandGroup* BuildShootBallSequence();
  // frc2::ParallelRaceGroup* RobotContainer::GenerateBallShootingSequence()
  frc2::SequentialCommandGroup* GenerateBallShootingSequence(int amountBalls);
  frc2::SequentialCommandGroup* Move(std::string Path);
  frc2::ParallelRaceGroup* BUCKLE();
  frc2::ParallelRaceGroup* MoveWithRetractedIntake(units::meter_t distance);
  frc2::ParallelRaceGroup* TurnWithRetractedIntake(
      units::degree_t amountToTurn);

  void AddTestButtonsToSmartDashboard();
  void AddTestTrajectoryCommandsToSmartDashboard();
  void AddAutonomousCommandsToSmartDashboard();
  void AddLightingCommandsToSmartDashboard();
  void ConfigureControllerButtonBindings();

  // Used when computing requested speeds for Tank Drive command.
  double GetDriveSpeedScalingFactor();

  // The robot's subsystems and commands are defined here.
 private:
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};
  frc::Joystick driverJoystick{0};
  frc::XboxController operatorController{1};

  Shooter m_shooter;
  Drivebase m_drivebase;
  Intake m_intake;
  Conveyor m_conveyor;
  Climber m_climber;
  Lighting m_lighting;
  IntakeDeployment m_intakeDeployment;
  TrajectoryCommandGenerator m_trajectoryGenerator;
  PhotonVision m_photonVision;

  frc::SendableChooser<frc2::Command*> m_autonomousOptions;
  // frc::SendableChooser<frc2::Command*> m_ClimberOptions;

  // variables

  bool isSwitched = false;
  frc::SlewRateLimiter<units::scalar> m_leftSpeedLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSpeedLimiter;
};
