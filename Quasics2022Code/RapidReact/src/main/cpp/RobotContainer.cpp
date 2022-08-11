// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "FORCOMPETITIONRobotConstants.h"
#include "TrajectoryCommandGenerator.h"
#include "commands/BreathingAllianceLights.h"
#include "commands/BreathingLights.h"
#include "commands/ConveyorTuningCommand.h"
#include "commands/Delay.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/DriveTuningCommand.h"
#include "commands/ExtendClimber.h"
#include "commands/ExtendIntake.h"
#include "commands/ExtendIntakeAuto.h"
#include "commands/MoveOneClimberArm.h"
#include "commands/MoveRobotTestCommand.h"
#include "commands/PatrioticLightsCmd.h"
#include "commands/RetractClimber.h"
#include "commands/RetractIntake.h"
#include "commands/RetractIntakeAtSpeedForTime.h"
#include "commands/RotateAtSpeedForDegrees.h"
#include "commands/RunConveyorAtSpeed.h"
#include "commands/RunConveyorAtSpeedForTime.h"
#include "commands/RunIntakeAtSpeed.h"
#include "commands/RunRearRollerAtSpeed.h"
#include "commands/RunShooterAtSpeed.h"
#include "commands/SetLightsToColor.h"
#include "commands/ShootForTime.h"
#include "commands/ShooterTuningCommand.h"
#include "commands/SimpleLightingCrawler.h"
#include "commands/TankDrive.h"
#include "commands/TriggerBasedShooterCommand.h"

// SUGGESTION TO FIX INTAKE FALLING DOWN
// MAKE LEFT AND RIGHT TRIGGERS ON DRIVER REMOTE TO BE EXTEND AND RETRACT INTAKE
// basically flip intake deployment and intake feed
// add control that will continue running intake deployment motor at low power
// after it is retracted basically reverse slew for retraction

// If defined, include various lighting commands on the smart dashboard.
#undef ENABLE_LIGHTING_CMDS

// If defined, set the default lighting command to be cycling red/white/blue.
#undef BE_PATRIOTIC

RobotContainer::RobotContainer()
    : m_trajectoryGenerator(
          // Drive base being controlled
          &m_drivebase,
          // Drive profile data
          {
              DriverConstants::ks,  // kS
              DriverConstants::kv,  // kV
              DriverConstants::ka   // kA
          },
          // PID configuration values
          {
              DriverConstants::kp,  // kP
              DriverConstants::ki,  // kI
              DriverConstants::kd   // kD
          }) {
  TankDrive tankDrive{
      &m_drivebase,
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();

        // Get the "base" speed value from the correct joystick
        double joystickValue;
        if (!isSwitched) {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        } else {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }

        // Return scaled speed.
        return scalingFactor * joystickValue;
      },
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();

        // Get the "base" speed value from the correct joystick
        double joystickValue;
        if (!isSwitched) {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        } else {
          joystickValue =
              +1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }

        // Return scaled speed.
        return scalingFactor * joystickValue;
      }};

  // Initialize all of your commands and subsystems here
  m_drivebase.SetDefaultCommand(tankDrive);

#ifdef BE_PATRIOTIC
  static PatrioticLightsCmd fourthOfJuly(&m_lighting, 3.0_s, true);
  m_lighting.SetDefaultCommand(fourthOfJuly);
#endif  // BE_PATRIOTIC

  TriggerBasedShooterCommand triggerBasedShooterCommand(&m_shooter,
                                                        &operatorController);
  m_shooter.SetDefaultCommand(triggerBasedShooterCommand);

  BreathingAllianceLights BreathingAllianceLights(&m_lighting, 0.50);
  m_lighting.SetDefaultCommand(BreathingAllianceLights);

  // Configure the button bindings
  ConfigureControllerButtonBindings();

  // Populate smart dashboard
  AddAutonomousCommandsToSmartDashboard();
  AddTestButtonsToSmartDashboard();
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);

  if (isTurbo) {
    return RobotValues::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotValues::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotValues::NORMAL_MODE_SPEED_SCALING;
  }
}

// Configure your button bindings here
void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&driverJoystick, logitechButtonId).WhileHeld(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, buttonId).WhileHeld(command);
}

// TODO: Configure other button bindings on driver and operator controllers.

void RobotContainer::ConfigureControllerButtonBindings() {
  static ExtendClimber extendClimber(&m_climber);
  static RetractClimber retractClimber(&m_climber);
  static RunIntakeAtSpeed runIntakeForward(&m_intake,
                                           RobotValues::INTAKE_FORWARD_SPEED);
  static RunIntakeAtSpeed runIntakeBackward(&m_intake,
                                            RobotValues::INTAKE_BACKWARD_SPEED);
  static RunConveyorAtSpeed conveyorUp(&m_conveyor,
                                       RobotValues::CONVEYOR_UP_SPEED);
  static RunConveyorAtSpeed conveyorDown(&m_conveyor,
                                         RobotValues::CONVEYOR_DOWN_SPEED);
  static RunShooterAtSpeed slowShoot(
      &m_shooter, RobotValues::SLOW_SHOOTER_SPEED,
      RobotValues::SLOW_SHOOTER_BACKROLLER_SPEED);  // no proper values for slow
                                                    // shoot
  static RunShooterAtSpeed fastShoot(
      &m_shooter, RobotValues::FAST_SHOOTER_SPEED,
      RobotValues::FAST_SHOOTER_BACKROLLER_SPEED);  // changed from 0.65
  static ExtendIntake extendIntake(&m_intakeDeployment,
                                   RobotValues::EXTEND_INTAKE_SPEED);
  static RetractIntake retractIntake(&m_intakeDeployment,
                                     RobotValues::RETRACT_INTAKE_SPEED);
  static ExtendIntake extendIntakeSLOW(&m_intakeDeployment, 0.40);
  static frc2::ParallelRaceGroup* buttonShootingHighGoal =
      ButtonShootingHighGoal();  // 0.40 Flywheel, 0.8 backroller
  static frc2::ParallelRaceGroup* buttonShootingLowGoal =
      ButtonShootingLowGoal();
  //   RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
  //                                      &extendClimber);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &extendIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &retractIntake);
  //   RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
  //                                      &retractClimber);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kLeftBumper,
                                     &conveyorDown);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kRightBumper,
                                     &conveyorUp);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     buttonShootingHighGoal);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kX,
                                     buttonShootingLowGoal);
  RunCommandWhenDriverButtonIsHeld(OperatorInterface::LogitechGamePad::Y_BUTTON,
                                   &extendClimber);
  RunCommandWhenDriverButtonIsHeld(OperatorInterface::LogitechGamePad::A_BUTTON,
                                   &retractClimber);
  RunCommandWhenDriverButtonIsHeld(
      OperatorInterface::LogitechGamePad::LEFT_TRIGGER, &runIntakeBackward);
  RunCommandWhenDriverButtonIsHeld(
      OperatorInterface::LogitechGamePad::RIGHT_TRIGGER, &runIntakeForward);
  RunCommandWhenDriverButtonIsHeld(OperatorInterface::LogitechGamePad::X_BUTTON,
                                   &extendIntakeSLOW);
}

void RobotContainer::AddTestButtonsToSmartDashboard() {
  // Basic drive base commands/tests
  frc::SmartDashboard::PutData(
      "Braking mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(true); },
                               {&m_drivebase}));
  frc::SmartDashboard::PutData(
      "Coasting mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(false); },
                               {&m_drivebase}));
  frc::SmartDashboard::PutData("1 Meter Forward",
                               new MoveRobotTestCommand(&m_drivebase, 0.35));
  frc::SmartDashboard::PutData("1 Meter Backward",
                               new MoveRobotTestCommand(&m_drivebase, -0.35));

  // Shooter Commands
  frc::SmartDashboard::PutData("Shooter Forward",
                               new RunShooterAtSpeed(&m_shooter, 0.6, 0.0));
  frc::SmartDashboard::PutData("Shooter Backward",
                               new RunShooterAtSpeed(&m_shooter, -0.6, 0.0));

  // Rear Roller Test commands
  frc::SmartDashboard::PutData("Roller Forward",
                               new RunRearRollerAtSpeed(&m_shooter, 0.35));
  frc::SmartDashboard::PutData("Roller Backward",
                               new RunRearRollerAtSpeed(&m_shooter, -0.35));

  // Intake commands (NOTE: values for extension and retraction are inverse
  //(-1 is extend at 100% speed)

  frc::SmartDashboard::PutData("Extend Intake 50%",
                               new ExtendIntake(&m_intakeDeployment, 0.5));
  frc::SmartDashboard::PutData("Retract Intake 50%",
                               new RetractIntake(&m_intakeDeployment, -0.75));

  frc::SmartDashboard::PutData("Intake Forward",
                               new RunIntakeAtSpeed(&m_intake, 1.00));
  frc::SmartDashboard::PutData("Intake Backward",
                               new RunIntakeAtSpeed(&m_intake, -1.00));

  // Intake deployment commands
  frc::SmartDashboard::PutData("Extend Intake auto",
                               new ExtendIntakeAuto(&m_intakeDeployment, 0.3));

  // Conveyor commands
  frc::SmartDashboard::PutData("Conveyor Forward",
                               new RunConveyorAtSpeed(&m_conveyor, 1.00));
  frc::SmartDashboard::PutData("Conveyor Backward",
                               new RunConveyorAtSpeed(&m_conveyor, -1.00));

  // Climber commands
  frc::SmartDashboard::PutData("Extend Climber", new ExtendClimber(&m_climber));
  frc::SmartDashboard::PutData("Retract Climber",
                               new RetractClimber(&m_climber));
  frc::SmartDashboard::PutData("Extend Right Climber",
                               new MoveOneClimberArm(&m_climber, false, true));
  frc::SmartDashboard::PutData("Retract Right Climber",
                               new MoveOneClimberArm(&m_climber, false, false));
  frc::SmartDashboard::PutData("Extend Left Climber",
                               new MoveOneClimberArm(&m_climber, true, true));
  frc::SmartDashboard::PutData("Retract Left Climber",
                               new MoveOneClimberArm(&m_climber, true, false));

  // Shooter tuning
  frc::SmartDashboard::PutData("Shooter tuning",
                               new ShooterTuningCommand(&m_shooter, 0.4));

  // Conveyor tuning
  frc::SmartDashboard::PutData("Conveyor Tuning",
                               new ConveyorTuningCommand(&m_conveyor, 0.0));

  // Drive tuning

  // frc::SmartDashboard::PutData("Drivebase Tuning", new
  // DriveTuningCommand(&m_drivebase, 0.0));
  // m_ClimberOptions.SetDefaultOption(
  //     "Do Nothing", new frc2::PrintCommand("I decline to do anything."));
  // m_ClimberOptions.AddOption("Extend Climber", new
  // ExtendClimber(&m_climber)); m_ClimberOptions.AddOption("Retract
  // Climber", new RetractClimber(&m_climber));

  // frc::SmartDashboard::PutData("Climber Options", &m_ClimberOptions);

#ifdef ENABLE_LIGHTING_CMDS
  // Lighting commands
  AddLightingCommandsToSmartDashboard();
#endif
}

void RobotContainer::AddLightingCommandsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "Alliance crawl", new SimpleLightingCrawler(&m_lighting, [](int pos) {
        frc::Color allianceColor;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
          allianceColor = frc::Color::kRed;
        } else if (frc::DriverStation::GetAlliance() ==
                   frc::DriverStation::kBlue) {
          allianceColor = frc::Color::kBlue;
        } else {
          allianceColor = frc::Color::kGreen;
        }
        constexpr auto cellCount = 5;
        constexpr auto groupSize = cellCount * 2;
        const auto posInGroup = pos % groupSize;
        return frc::Color(posInGroup < cellCount ? frc::Color::kBlack
                                                 : allianceColor);
      }));
  frc::SmartDashboard::PutData("Crawling Lights",
                               new SimpleLightingCrawler(&m_lighting));
  frc::SmartDashboard::PutData(
      "4th of July down", new PatrioticLightsCmd(&m_lighting, 3.0_s, true));
  frc::SmartDashboard::PutData(
      "4th of July up", new PatrioticLightsCmd(&m_lighting, 3.0_s, false));
  frc::SmartDashboard::PutData(
      "Set All ligths to Red",
      new SetLightsToColor(&m_lighting, Lighting::StockColor::Red));
  frc::SmartDashboard::PutData(
      "Breathing Lights", new BreathingLights(&m_lighting, 0, 255, 0, 0.75));
  frc::SmartDashboard::PutData("Alliance Breathing Lights",
                               new BreathingAllianceLights(&m_lighting, 0.75));
}

void RobotContainer::AddTestTrajectoryCommandsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "MaeTestVerySlow",
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "MaeTestVerySlow.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "MaeStartAndGrabTopComeBack",
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "MaeStartAndGrabTopComeBack.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 1,1 -> intermediate = 0,1",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps, 0.4_mps_sq},  // changed values from 0.5 and 0.8 to
                                     // 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(0_m, 1_m)},
          frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 1,0 -> intermediate = 1,0",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps,
              0.4_mps_sq},  // changed values from 0.5 and 0.8 to 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m)},
          frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 0,0 -> intermediate = (1.5,0), "
      "(1.5,1.5), "
      "(0,1.5)",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps,
              0.4_mps_sq},  // changed values from 0.5 and 0.8 to 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(1.5_m, 0_m),
                                          frc::Translation2d(1.5_m, 1.5_m),
                                          frc::Translation2d(0_m, 1.5_m)},
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard() {
  m_autonomousOptions.SetDefaultOption(
      "Do Nothing", new frc2::PrintCommand("I decline to do anything."));

#ifdef ENABLE_TEST_CMDS_IN_AUTO
  m_autonomousOptions.AddOption("Shoot @ 20% for 3 seconds",
                                new ShootForTime(&m_shooter, 0.2, 3_s));

  m_autonomousOptions.AddOption("Shoot @ 20% for 2sec, move @ 20% for 1m",
                                BuildShootAndMoveCommand(0.2, 2_s, 0.2, 1_m));
#endif
  m_autonomousOptions.AddOption(
      "Just Move", new DriveAtPowerForMeters(&m_drivebase, -0.50, 2_m));
  m_autonomousOptions.AddOption("Just shoot", GenerateBallShootingSequence(1));
  m_autonomousOptions.AddOption("Red - Shoot/Move", RSM2Manual());
  m_autonomousOptions.AddOption("Blue - Shoot/Move", BSM4Manual());
  m_autonomousOptions.AddOption("Get1Shoot2 Face Side Wall",
                                Pickup1Shoot2FacingSide());
  m_autonomousOptions.AddOption("Get1Shoot2 Face Bar Hang",
                                Pickup1Shoot2FacingClimbers());
  m_autonomousOptions.AddOption("Get1Shoot2 Face Human Player",
                                Pickup1Shoot2FacingHuman());

#ifdef ENABLE_TRAJECTORIES_IN_AUTO
  m_autonomousOptions.AddOption("RSM1", RSM1());
  m_autonomousOptions.AddOption("RSM2", RSM2());
  m_autonomousOptions.AddOption("BSM3", BSM3());
  m_autonomousOptions.AddOption("BSM4", BSM4());
  m_autonomousOptions.AddOption("RSP1", RSP1());
  m_autonomousOptions.AddOption("RSP3", RSP3());
  m_autonomousOptions.AddOption("BSP4", BSP4());
  m_autonomousOptions.AddOption("BSP6", BSP6());
  m_autonomousOptions.AddOption("RSPS1", RSPS1());
  m_autonomousOptions.AddOption("BSPS2", BSPS2());
  m_autonomousOptions.AddOption("RPSPS1", RPSPS1());
  m_autonomousOptions.AddOption("BPSPS2", BPSPS2());
  m_autonomousOptions.AddOption("RALL1", RALL1());
  m_autonomousOptions.AddOption("BALL2", BALL2());
#endif  // ENABLE_TRAJECTORIES_IN_AUTO

  frc::SmartDashboard::PutData("Auto mode", &m_autonomousOptions);
}

//
/// Commands For the xB Button on the xbox controller should start shooter,
/// delay intake and move intake upwards

frc2::ParallelRaceGroup* RobotContainer::ButtonShootingHighGoal() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<ShootForTime>(
      &m_shooter, RobotValues::FAST_SHOOTER_SPEED, 3.5_s,
      RobotValues::FAST_SHOOTER_BACKROLLER_SPEED));  // changed from 0.65
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(ConveyorDelay())));
  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::ButtonShootingLowGoal() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<ShootForTime>(
      &m_shooter, RobotValues::SLOW_SHOOTER_SPEED, 3.5_s,
      RobotValues::SLOW_SHOOTER_BACKROLLER_SPEED));  // needs adjusted and
                                                     // finetuned
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(ConveyorDelay())));
  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::ConveyorDelay() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<Delay>(0.5_s));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 0.9, 3_s));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::ConveyorRetractionDelay() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<Delay>(0.4_s));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, -0.9, 0.1_s));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 0.9, 2_s));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BuildShootAndMoveCommand(
    double powerShoot, units::second_t timeShoot, double powerMove,
    units::meter_t distanceToMove) {
  // Holds the sequence of the commands to be executed as a group.
  std::vector<std::unique_ptr<frc2::Command>> commands;

  // Add each of the individual commands to the sequence.
  commands.push_back(
      std::make_unique<frc2::PrintCommand>("Starting 'shooting' sequence"));
  commands.push_back(
      std::make_unique<ShootForTime>(&m_shooter, powerShoot, timeShoot));
  commands.push_back(
      std::make_unique<frc2::PrintCommand>("Shooting completed"));
  commands.push_back(std::make_unique<frc2::PrintCommand>("Moving away"));
  commands.push_back(std::make_unique<DriveAtPowerForMeters>(
      &m_drivebase, powerMove, distanceToMove));
  commands.push_back(
      std::make_unique<frc2::PrintCommand>("Sequence completed"));

  // Builds the command group object.
  return new frc2::SequentialCommandGroup(std::move(commands));
}

// current autonomous commands that are manual
frc2::SequentialCommandGroup* RobotContainer::ConveyorShootingAuto() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::RSM2Manual() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, -0.35, -2.8_m));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSM4Manual() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, -0.35, -2.8_m));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

frc2::SequentialCommandGroup* RobotContainer::Pickup1Shoot2FacingSide() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(BUCKLE())));
  commands.push_back(
      std::make_unique<ExtendIntakeAuto>(&m_intakeDeployment, 0.2));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(BuildMaualDrivePickup(1.2116_m))));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 1.00, 0.3_s));
  commands.push_back(
      std::make_unique<RotateAtSpeedForDegrees>(&m_drivebase, 0.4, 180_deg));
  commands.push_back(std::make_unique<RetractIntakeAtSpeedForTime>(
      &m_intakeDeployment, 0.4, 0.77_s));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(2_m))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(TurnWithRetractedIntake(20.84_deg))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(1.031_m))));
  // test how much time it takes intake to go up put here
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

// for Robot Sequence that is center of pentagon
frc2::SequentialCommandGroup* RobotContainer::Pickup1Shoot2FacingClimbers() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(BUCKLE())));
  commands.push_back(
      std::make_unique<ExtendIntakeAuto>(&m_intakeDeployment, 0.2));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(BuildMaualDrivePickup(1.2116_m))));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 1.00, 0.3_s));
  commands.push_back(
      std::make_unique<RotateAtSpeedForDegrees>(&m_drivebase, 0.4, 180_deg));
  commands.push_back(std::make_unique<RetractIntakeAtSpeedForTime>(
      &m_intakeDeployment, 0.4, 0.77_s));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(2_m))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(TurnWithRetractedIntake(-20.84_deg))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(1.031_m))));
  // same thing for here
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

// for Robot Sequence on edge of Pentagon
frc2::SequentialCommandGroup* RobotContainer::Pickup1Shoot2FacingHuman() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(BUCKLE())));
  commands.push_back(
      std::make_unique<ExtendIntakeAuto>(&m_intakeDeployment, 0.2));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(BuildMaualDrivePickup(1.2116_m))));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 1.00, 0.3_s));
  commands.push_back(
      std::make_unique<RotateAtSpeedForDegrees>(&m_drivebase, 0.4, 90_deg));
  commands.push_back(std::make_unique<RetractIntakeAtSpeedForTime>(
      &m_intakeDeployment, 0.4, 0.77_s));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(0.9116_m))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(TurnWithRetractedIntake(90_deg))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(1.3_m))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(TurnWithRetractedIntake(30.84_deg))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(MoveWithRetractedIntake(1.031_m))));
  // and here
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

// shoot pick up 2 shoot
// all rotations are clockwise
//  go backwards 1.08 meters
//  rotate 135
//  move forwards 1.395
//  rotate 135ish
//  move forwards 2.857
// rotate 180
// move 0.9116 this and the rest are stuff from the previuos
//  use steps in edge to complete the rest

frc2::ParallelRaceGroup* RobotContainer::BuildMaualDrivePickup(
    units::meter_t distance) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<RunIntakeAtSpeed>(&m_intake, 1.00));
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, 0.50, distance));
  return new frc2::ParallelRaceGroup(std::move(commands));
}
// autonomous sequences
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
// shoot and move
//
frc2::SequentialCommandGroup* RobotContainer::RSM1() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(Move("RSM1"))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::RSM2() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(Move("RSM2"))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSM3() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(Move("BSM3"))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSM4() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(Move("BSM4"))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}
//
//
//  pick up 1 then shoot
//
frc2::SequentialCommandGroup* RobotContainer::RSP1() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RSP1(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RSP1(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::RSP2() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RSP2(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RSP2(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}  // paths no made yet

frc2::SequentialCommandGroup* RobotContainer::RSP3() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RSP3(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RSP3(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSP4() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BSP4(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BSP4(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSP5() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BSP5(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BSP5(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}  // paths no made yet

frc2::SequentialCommandGroup* RobotContainer::BSP6() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BSP6(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BSP6(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}
//
// shoot 1 pickup 2 shoot 2

// needs to be a new trajectory after every ball pickup in order
// tell the conveyor when to move
frc2::SequentialCommandGroup* RobotContainer::RSPS1() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RSPS1(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RSPS1(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RSPS1(Part3)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BSPS2() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BSPS2(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BSPS2(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BSPS2(Part3)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}
//
//
// Shoots 4 balls

frc2::SequentialCommandGroup* RobotContainer::RPSPS1() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("RPSPS1(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RPSPS1(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("RPSPS1(Part3)"))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("RPSPS1(Part4)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RPSPS1(Part5)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BPSPS2() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("BPSPS2(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BPSPS2(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("BPSPS2(Part3)"))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("BPSPS2(Part4)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BPSPS2(Part5)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}
//
//
// Shoots 5 balls

frc2::SequentialCommandGroup* RobotContainer::RALL1() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RALL1(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RALL1(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RALL1(Part3)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RALL1(Part4)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("RALL1(Part5)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("RALL1(Part6)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BALL2() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BALL2(Part1)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BALL2(Part2)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BALL2(Part3)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BALL2(Part4)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivePickUpPositionBall("BALL2(Part5)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(DrivingBackToShoot("BALL2(Part6)"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
// example sequence for the autonomous part
frc2::SequentialCommandGroup* RobotContainer::BuildExampleAutonomousCommand() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(1))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("PickUpTwoBallSallyPart1"))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivePickUpPositionBall("PickUpTwoBallSallyPart1"))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      DrivingBackToShoot("PickUpTwoBallSallyPart2"))));
  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(GenerateBallShootingSequence(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

// helper functions for autonomous

frc2::ParallelCommandGroup* RobotContainer::DrivingBackToShoot(
    std::string Path) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  Path = Path + ".wpilib.json";
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          Path, TrajectoryCommandGenerator::TelemetryHandling::
                    ResetTelemetryAtStart))));
  commands.push_back(std::make_unique<RetractIntakeAtSpeedForTime>(
      &m_intakeDeployment, 0.7, 0.3_s));

  return new frc2::ParallelCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::DrivePickUpPositionBall(
    std::string Path) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  Path = Path + ".wpilib.json";
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(DrivingAndPickingUpBall(Path))));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 0.8, 0.3_s));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::DrivingAndPickingUpBall(
    std::string Path) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          Path, TrajectoryCommandGenerator::TelemetryHandling::
                    ResetTelemetryAtStart))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(PickingUpBall())));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::PickingUpBall() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::make_unique<ExtendIntakeAuto>(&m_intakeDeployment, 0.2));
  commands.push_back(std::make_unique<RunIntakeAtSpeed>(&m_intake, 0.8));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

/* frc2::ParallelRaceGroup* RobotContainer::GenerateBallShootingSequence(){
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::move(std::unique_ptr<frc2::Command>(BuildShootBallSequence())));
change to the command name below
      // commands.push_back(std::make_unique<RetractIntake>(
  //     &m_intakeDeployment, 0.15));
}
*/

// add parallel race group so that the intake stays up
frc2::SequentialCommandGroup* RobotContainer::GenerateBallShootingSequence(
    int amountBalls) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  int count = amountBalls;
  while (count >= 0) {
    commands.push_back(
        std::move(std::unique_ptr<frc2::Command>(BuildShootBallSequence())));
    count--;
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}
frc2::ParallelCommandGroup* RobotContainer::BuildShootBallSequence() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<ShootForTime>(
      &m_shooter, 0.6, 1.5_s,
      0.35));  // might need to change this to 0.6 to match the rest
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 0.8, 1.5_s));
  return new frc2::ParallelCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::Move(std::string Path) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  Path = Path + ".wpilib.json";
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          Path, TrajectoryCommandGenerator::TelemetryHandling::
                    ResetTelemetryAtStart))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autonomousOptions.GetSelected();
}

frc2::ParallelRaceGroup* RobotContainer::BUCKLE() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, -0.50, -0.3_m));
  commands.push_back(
      std::make_unique<RetractIntake>(&m_intakeDeployment, -0.1));
  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::MoveWithRetractedIntake(
    units::meter_t distance) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  double power;
  if (distance >= units::meter_t(0)) {
    power = 0.50;
  } else {
    power = -0.50;
  }
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, power, distance));
  commands.push_back(
      std::make_unique<RetractIntake>(&m_intakeDeployment, -0.2));
  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::TurnWithRetractedIntake(
    units::degree_t amountToTurn) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<RotateAtSpeedForDegrees>(
      &m_drivebase, 0.4, amountToTurn));
  commands.push_back(
      std::make_unique<RetractIntake>(&m_intakeDeployment, -0.2));
  return new frc2::ParallelRaceGroup(std::move(commands));
}
