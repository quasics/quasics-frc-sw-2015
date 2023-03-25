// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>
#include <list>

#include "Constants.h"
#include "commands/Autos.h"
#include "commands/PauseRobot.h"
#include "commands/SetCubeOrConeIntakeSpeed.h"
#include "commands/clamp/ClampWithIntake.h"
#include "commands/clamp/ClampWithIntakeAtSpeedForTime.h"
#include "commands/examples/BlackAndWhiteLights.h"
#include "commands/examples/SplitLightingExample.h"
#include "commands/floor/AutoFloorRetract.h"
#include "commands/floor/MoveFloorEjection.h"
#include "commands/floor/MoveFloorEjectionAtPowerForTime.h"
#include "commands/floor/ShootTheGamePiece.h"
#include "commands/intake/ExhaustWithRoller.h"
#include "commands/intake/ExhaustWithRollerAtSpeedForTime.h"
#include "commands/intake/ExtendIntake.h"
#include "commands/intake/ExtendIntakeAtSpeedForTime.h"
#include "commands/intake/IntakeWithRoller.h"
#include "commands/intake/IntakeWithRollerAtSpeedForTime.h"
#include "commands/intake/ReleaseWithIntake.h"
#include "commands/intake/ReleaseWithIntakeAtSpeedForTime.h"
#include "commands/intake/RetractIntake.h"
#include "commands/intake/RetractIntakeAtSpeedForTime.h"
#include "commands/intake/RunIntakeCubeOrConeToggleCommand.h"
#include "commands/intake/TriggerBasedRollerCommand.h"
#include "commands/lighting/SetLightsToColor.h"
#include "commands/movement/AprilTagDriveToTarget.h"
#include "commands/movement/ArcadeDrive.h"
#include "commands/movement/DriveAtPowerForMeters.h"
#include "commands/movement/DriveUntilPitchAngleChange.h"
#include "commands/movement/RotateAtAngle.h"
#include "commands/movement/SelfBalancing.h"
#include "commands/movement/StraightLineDriving.h"
#include "commands/movement/TankDrive.h"
#include "commands/movement/ToggleBrakingMode.h"
#include "commands/movement/TurnDegreesImported.h"

#undef NormalDriving
#define splitStickMovement

RobotContainer::RobotContainer()
    : m_trajectoryGenerator(
          // Drive base being controlled
          &m_drivebase,
          // Drive profile data
          {
              PathWeaverConstants::kS,  // kS
              PathWeaverConstants::kV,  // kV
              PathWeaverConstants::kA   // kA
          },
          // PID configuration values
          {
              PathWeaverConstants::kP,  // kP
              PathWeaverConstants::kI,  // kI
              PathWeaverConstants::kD   // kD
          }),
      m_leftSlewRateLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT},
      m_rightSlewRateLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT} {
  // Initialize all of your commands and subsystems here

#ifdef NormalDriving
  TankDrive tankDrive{
      &m_drivebase,
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double joystickValue;

        if (isInverted) {
          joystickValue = -1 * scalingFactor *
                          m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        } else {
          joystickValue = +1 * scalingFactor *
                          m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }
        return m_leftSlewRateLimiter.Calculate(joystickValue);
      },
      [this] {
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double joystickValue;

        if (isInverted) {
          joystickValue = -1 * scalingFactor *
                          m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        } else {
          joystickValue = +1 * scalingFactor *
                          m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }
        return m_rightSlewRateLimiter.Calculate(joystickValue);
      }};

  m_drivebase.SetDefaultCommand(tankDrive);
#endif

#ifdef splitStickMovement
  ArcadeDrive arcadeDrive{
      &m_drivebase,
      [this] {
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double powertoRobot;

        if (isInverted)
          powertoRobot = (-1) * scalingFactor *
                         m_driverStick.GetRawAxis(
                             OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);

        else
          powertoRobot = scalingFactor *
                         m_driverStick.GetRawAxis(
                             OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);

        return m_leftSlewRateLimiter.Calculate(powertoRobot);
      },
      [this] {
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double rotationtoRobot;

        if (isInverted)
          rotationtoRobot =
              (-1) * scalingFactor * 0.75 *
              m_driverStick.GetRawAxis(
                  OperatorInterface::LogitechGamePad::RIGHT_X_AXIS);

        else
          rotationtoRobot =
              scalingFactor * 0.75 *
              m_driverStick.GetRawAxis(
                  OperatorInterface::LogitechGamePad::RIGHT_X_AXIS);
        return rotationtoRobot;
      }};

  m_drivebase.SetDefaultCommand(arcadeDrive);
#endif

#ifdef ENABLE_ROLLER_INTAKE_MOTORS
  TriggerBasedRollerCommand triggerBasedRollerCommand(
      &m_intakeRoller, &m_configSettings, &m_operatorController);

  m_intakeRoller.SetDefaultCommand(triggerBasedRollerCommand);
#endif

  // Configure the button bindings
  ConfigureControllerButtonBindings();
  AddTestButtonsToSmartDashboard();
  AddTeamAndStationSelectorToSmartDashboard();
  AddRobotSequenceSelectorToSmartDashboard();
}

void RobotContainer::setInverted(bool invert) {
  isInverted = invert;
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);

  if (isTurbo) {
    return RobotSpeedScaling::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotSpeedScaling::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotSpeedScaling::NORMAL_MODE_SPEED_SCALING;
  }
}

void RobotContainer::RunCommandWhileDriverButtonIsHeld(int logitechButtonId,
                                                       frc2::Command *command) {
  frc2::JoystickButton(&m_driverStick, logitechButtonId).WhileTrue(command);
}

void RobotContainer::RunCommandWhenDriverButtonIsPressed(
    int logitechButtonId, frc2::Command *command) {
  frc2::JoystickButton(&m_driverStick, logitechButtonId)
      .Debounce(100_ms, frc::Debouncer::DebounceType::kBoth)
      .OnTrue(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command *command) {
  frc2::JoystickButton(&m_operatorController, buttonId).WhileTrue(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsPressed(
    int buttonId, frc2::Command *command) {
  frc2::JoystickButton(&m_operatorController, buttonId)
      .Debounce(100_ms, frc::Debouncer::DebounceType::kBoth)
      .OnTrue(command);
}

// ToggleOnTrue command can be used or should this be in each individual command

void RobotContainer::ConfigureControllerButtonBindings() {
  static ExtendIntake extendIntake(&m_intakeDeployment, 0.30);
  static RetractIntake retractIntake(&m_intakeDeployment, 0.50);
  // static ClampWithIntake clampWithIntake(&m_intakeClamp, 0.5); NOT NEEDED
  // static ReleaseWithIntake releaseWithIntake(&m_intakeClamp, 0.5); NOT NEEDED
  static ExhaustWithRoller exhaustWithRoller(&m_intakeRoller, 0.85);
  static MoveFloorEjection moveFloor(&m_floorEjection, 0.2);
  static AutoFloorRetract resetFloorEjection(&m_floorEjection, 0.4);
  // static ShootTheGamePiece shootPiece(&m_floorEjection, 45,0.3);  // Not
  // Working Yet
  static MoveFloorEjectionAtPowerForTime shootPiece(&m_floorEjection, 0.45,
                                                    0.25_s);
  static SelfBalancing selfBalancing(&m_drivebase);
  static ToggleBrakingMode toggleBrakingMode(&m_drivebase);
  static SetCubeOrConeIntakeSpeed toggleCubeOrCone(&m_configSettings);
  static StraightLineDriving straightLineDriving(&m_drivebase, &m_driverStick);
  static frc2::PrintCommand placeholder("Doing something!!!!");

  // Rollers Controlled by Command TriggerBasedRollerCommand
  RunCommandWhenOperatorButtonIsPressed(frc::XboxController::Button::kX,
                                        &toggleCubeOrCone);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &retractIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &extendIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kLeftBumper,
                                     &moveFloor);
  RunCommandWhenOperatorButtonIsPressed(
      frc::XboxController::Button::kRightBumper, &resetFloorEjection);
  RunCommandWhenOperatorButtonIsPressed(frc::XboxController::Button::kB,
                                        &shootPiece);

  RunCommandWhenDriverButtonIsPressed(
      OperatorInterface::LogitechGamePad::B_BUTTON, &toggleBrakingMode);
  /*RunCommandWhenDriverButtonIsPressed(
      OperatorInterface::LogitechGamePad::A_BUTTON,
      &straightLineDriving);  UNTESTED*/
  RunCommandWhileDriverButtonIsHeld(
      OperatorInterface::LogitechGamePad::Y_BUTTON, &selfBalancing);
  RunCommandWhenDriverButtonIsPressed(
      OperatorInterface::LogitechGamePad::BACK_BUTTON,
      new frc2::InstantCommand([this]() { isInverted = !isInverted; }));
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  frc2::Command *selectedOperation =
      m_RobotSequenceAutonomousOptions.GetSelected();
  frc2::Command *teamAndPosCmd =
      m_TeamAndStationAutonomousOptions.GetSelected();
  if (selectedOperation == nullptr || teamAndPosCmd == nullptr) {
    // This shouldn't happen if things were set up right.  But it did.  So they
    // weren't. We'll bail out, but at least return a valid pointer that will
    // tell us something went wrong when it's run.
    static frc2::PrintCommand somethingIsScrewyCommand(
        "Selection error: can't decide what to do");
    return &somethingIsScrewyCommand;
  }

  std::string operationName = selectedOperation->GetName();
  std::string teamAndPosName = teamAndPosCmd->GetName();

  return AutonomousCommands::GetAutonomousCommand(
      &m_drivebase, &m_intakeDeployment, &m_intakeRoller, &m_floorEjection,
      operationName, teamAndPosName);
}

void RobotContainer::AddTestButtonsToSmartDashboard() {
  // Sample lighting commands.
  frc::SmartDashboard::PutData("Black&White",
                               new BlackAndWhiteLights(&m_lighting));
  frc::SmartDashboard::PutData(
      "Split Lighting",
      new SplitLightingExample(&m_lighting, Lighting::RED, Lighting::BLUE));

  if (false) {
    frc::SmartDashboard::PutData("Red lights",
                                 new SetLightsToColor(&m_lighting, 255, 0, 0));
    frc::SmartDashboard::PutData("Blue lights",
                                 new SetLightsToColor(&m_lighting, 0, 0, 255));
  }

  frc::SmartDashboard::PutData(
      "Reset Encoder",
      new frc2::InstantCommand([this]() { m_floorEjection.ResetEncoder(); },
                               {&m_floorEjection}));
  frc::SmartDashboard::PutData(
      "Set Coasting Mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(false); },
                               {&m_drivebase}));

  frc::SmartDashboard::PutData(
      "Set Breaking Mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(true); },
                               {&m_drivebase}));

  frc::SmartDashboard::PutData(
      "Set inverted",
      new frc2::InstantCommand([this]() { setInverted(true); }));

  frc::SmartDashboard::PutData(
      "Set not inverted",
      new frc2::InstantCommand([this]() { setInverted(false); }));

  frc::SmartDashboard::PutData("Test Command", TESTCOMMAND());

  frc::SmartDashboard::PutData("PATHWEAVER TEST COMMAND", TestPathCommand());

  if (false) {
    // AprilTag test commands
    frc::SmartDashboard::PutData(
        "Drive To April Tag",
        new AprilTagDriveToTarget(&m_photonLibVision, &m_drivebase, 3));
  }

  frc::SmartDashboard::PutData("Autonmous Floor Retraction",
                               new AutoFloorRetract(&m_floorEjection, 0.1));
  /*
    frc::SmartDashboard::PutData(
        "Shoot 70 for 0.15",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.7, 0.15_s));
    frc::SmartDashboard::PutData(
        "Shoot 70 for 0.1",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.7, 0.1_s));
    frc::SmartDashboard::PutData(
        "Shoot 80 for 0.1",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.8, 0.1_s));
    frc::SmartDashboard::PutData(
        "Shoot 90 for 0.1",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.9, 0.1_s));*/
  frc::SmartDashboard::PutData("YEET", new MoveFloorEjectionAtPowerForTime(
                                           &m_floorEjection, 1.00, 0.2_s));
  frc::SmartDashboard::PutData(
      "Shoot 50 for 0.1",
      new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.5, 0.1_s));
  frc::SmartDashboard::PutData(
      "Shoot 30 for 0.25",
      new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.3, 0.25_s));
  frc::SmartDashboard::PutData(
      "Shoot 35 for 0.25",
      new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.35, 0.25_s));
  frc::SmartDashboard::PutData(
      "Shoot 40 for 0.2",
      new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.4, 0.2_s));
  frc::SmartDashboard::PutData(
      "Shoot 50 for 0.2",
      new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 0.5, 0.2_s));

  /*
    frc::SmartDashboard::PutData(
        "Curiousity",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 1.00, 0.075_s));
    frc::SmartDashboard::PutData(
        "Curiousity lower",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, 1.00, 0.065_s));
    frc::SmartDashboard::PutData("Safety", new MoveFloorEjectionAtPowerForTime(
                                               &m_floorEjection, 0.1, 0.75_s));
    frc::SmartDashboard::PutData(
        "Safety Back",
        new MoveFloorEjectionAtPowerForTime(&m_floorEjection, -0.1, 1_s));

    frc::SmartDashboard::PutData(
        "Intake Extension Timed",
        new ExtendIntakeAtSpeedForTime(&m_intakeDeployment, 0.3, 0.2_s));
    frc::SmartDashboard::PutData(
        "Intake Retraction Timed",
        new RetractIntakeAtSpeedForTime(&m_intakeDeployment, 0.5, 0.2_s));
    frc::SmartDashboard::PutData("Intake Extension",
                                 new ExtendIntake(&m_intakeDeployment, 0.3));
    frc::SmartDashboard::PutData("Intake Retraction",
                                 new ExtendIntake(&m_intakeDeployment, 0.5));
    */
}

frc2::Command *BuildNamedPrintCommand(std::string name, std::string text = "") {
  if (text.empty()) {
    text = name;
  }
  frc2::Command *cmd = new frc2::PrintCommand(text);
  cmd->SetName(name);
  return cmd;
}

void AddNamedCommandToSelector(frc::SendableChooser<frc2::Command *> &selector,
                               std::string name, std::string text = "") {
  selector.AddOption(name, BuildNamedPrintCommand(name, text));
}

void AddingNamedPositionsToSelectorWithLoop(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultTeamsAndPositionsList{
          {AutonomousTeamAndStationPositions::Blue2, "Blue 2"},
          {AutonomousTeamAndStationPositions::Blue3, "Blue 3"},
          {AutonomousTeamAndStationPositions::Red1, "Red 1"},
          {AutonomousTeamAndStationPositions::Red2, "Red 2"},
          {AutonomousTeamAndStationPositions::Red3, "Red 3"},
      };

  for (auto &[name, text] : nonDefaultTeamsAndPositionsList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void AddingNamedAutonomousSequencesToSelectorWithLoop(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultAutonomousSequenceList{
          {AutonomousSelectedOperation::DropAndCharge,
           "Drop the Game Piece then Charge"},
          {AutonomousSelectedOperation::DropAndGTFO,
           "Drop the Game Piece then Get Out of Community"},
          {AutonomousSelectedOperation::DropGamePiece,
           "Just drop the Game Piece"},
          {AutonomousSelectedOperation::GTFO, "Just Get out of the Community"},
          {AutonomousSelectedOperation::GTFODock,
           "Get Out of the Community then Get on the Charging Station"},
          {AutonomousSelectedOperation::JustCharge,
           "Just Get on the Starting Station"},
          {AutonomousSelectedOperation::MoveToDefenseAgainstScoringWall,
           "Get out of the Community and Move to Defensive Position by "
           "hugging "
           "the scoring wall"},
          {AutonomousSelectedOperation::MoveToDefenseAgainstOuterWall,
           "Get out of the community and move to defensive position by "
           "hugging "
           "the outer wall"},
          {AutonomousSelectedOperation::ScorePiece, "Score The game Piece"},
          {AutonomousSelectedOperation::ScoreAndMoveToDefenseAgainstScoringWall,
           "Score a piece then move to defensive position by hugging "
           "the scoring wall"},
          {AutonomousSelectedOperation::ScoreAndMoveToDefenseAgainstOuterWall,
           "Score a piece then move to defensive position by hugging "
           "the outer wall"},
          {AutonomousSelectedOperation::DropAndMoveToDefenseAgainstScoringWall,
           "Drop a piece then move to defensive position by hugging "
           "the scoring wall"},
          {AutonomousSelectedOperation::DropAndMoveToDefenseAgainstOuterWall,
           "Drop a piece then move to defensive position by hugging "
           "the outer wall"},
          {AutonomousSelectedOperation::ScoreThenCharge,
           "Score the Game Piece then Get on the Charging Station"},
          {AutonomousSelectedOperation::ScoreThenEndNearGamePiece,
           "Score then End next to a Game Piece"},
          {AutonomousSelectedOperation::ScoreGTFOCharge,
           "Score, GTFO then Charge"},
          {AutonomousSelectedOperation::ScoreTwiceThenCharge,
           "Score, get another piece and score that, then charge"}};

  for (auto &[name, text] : nonDefaultAutonomousSequenceList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard() {
  m_TeamAndStationAutonomousOptions.SetDefaultOption(
      AutonomousTeamAndStationPositions::Blue1,
      BuildNamedPrintCommand(AutonomousTeamAndStationPositions::Blue1,
                             "Blue 1"));

  AddingNamedPositionsToSelectorWithLoop(m_TeamAndStationAutonomousOptions);

  frc::SmartDashboard::PutData("Team and Station Auto Selector",
                               &m_TeamAndStationAutonomousOptions);
}

void RobotContainer::AddRobotSequenceSelectorToSmartDashboard() {
  m_RobotSequenceAutonomousOptions.SetDefaultOption(
      AutonomousSelectedOperation::DoNothing,
      BuildNamedPrintCommand(AutonomousSelectedOperation::DoNothing,
                             "Do Nothing"));

  AddingNamedAutonomousSequencesToSelectorWithLoop(
      m_RobotSequenceAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Sequence Auto Selector",
                               &m_RobotSequenceAutonomousOptions);
}

frc2::SequentialCommandGroup *RobotContainer::TESTCOMMAND() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, 0.4, 1_m));
  commands.push_back(
      std::make_unique<TurnDegreesImported>(&m_drivebase, 0.5, 90_deg));
  commands.push_back(std::make_unique<PauseRobot>(&m_drivebase, 0.1_s));
  commands.push_back(
      std::make_unique<DriveAtPowerForMeters>(&m_drivebase, 0.4, 1_m));
  // Builds the command group object.
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup *RobotContainer::TestPathCommand() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "GTFODockB1.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

void RobotContainer::Periodic() {
  // Do something.....
}