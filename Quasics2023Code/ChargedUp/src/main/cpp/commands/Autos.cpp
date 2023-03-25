#include "commands/Autos.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>
#include <string>

#include "Constants.h"
#include "commands/PauseRobot.h"
#include "commands/floor/MoveFloorEjectionAtPowerForTime.h"
#include "commands/intake/AutoIntakeExtension.h"
#include "commands/intake/ExhaustWithRoller.h"
#include "commands/intake/ExhaustWithRollerAtSpeedForTime.h"
#include "commands/intake/ExtendIntakeAtSpeedForTime.h"
#include "commands/intake/RetractIntakeAtSpeedForTime.h"
#include "commands/movement/DriveAtPowerForMeters.h"
#include "commands/movement/DriveUntilPitchAngleChange.h"
#include "commands/movement/RotateAtAngle.h"
#include "commands/movement/SelfBalancing.h"
#include "commands/movement/TurnDegreesImported.h"

namespace AutonomousCommands {
namespace Helpers {
  //
  // Implementation of all of the helper functions
  //
  frc2::Command *DropGamePieceHelperCommand(FloorEjection *floorEjection) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(new MoveFloorEjectionAtPowerForTime(
            floorEjection, AutonomousSpeeds::DROP_FLOOR_EJECTION_SPEED,
            AutonomousSpeeds::DROP_FLOOR_EJECTION_TIME)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreGamePieceHelperCommand(FloorEjection *floorEjection) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(new MoveFloorEjectionAtPowerForTime(
            floorEjection, AutonomousSpeeds::SCORE_FLOOR_EJECTION_SPEED,
            AutonomousSpeeds::SCORE_FLOOR_EJECTION_TIME)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::ParallelRaceGroup *MoveAndIntake(Drivebase *drivebase,
                                         IntakeRoller *intakeRoller) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 0.5_m)));
    commands.push_back(std::unique_ptr<frc2::Command>(new ExhaustWithRoller(
        intakeRoller, IntakeConstants::RollerSpeeds::CUBES)));

    return new frc2::ParallelRaceGroup(std::move(commands));
  }

  frc2::Command *GamePiecePickupHelperCommand(Drivebase *drivebase,
                                              IntakeRoller *intakeRoller) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(  // TODO fix this
        std::unique_ptr<frc2::Command>(MoveAndIntake(drivebase, intakeRoller)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *GTFODOCK(Drivebase *drivebase, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      // In this case, we need to move back out of the community area (for the
      // mobility points), and then move forward and balance on the charging
      // station.
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
              drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.5_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{
              drivebase, 0.5}));  // LOOK INTO HOW TO DO OR
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing{drivebase}));
    } else {
      // In this case, we need to move back out of the community area (for the
      // mobility points), then turn and drive until we're in line with the
      // middle of the charging station, and then move forward and balance on
      // the charging station.
      const bool firstTurnIsCounterClockwise =
          (teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
           teamAndPosName == AutonomousTeamAndStationPositions::Red1);
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
              drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.0_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
      commands.push_back(
          std::make_unique<PauseRobot>(drivebase, 0.1_s));  // ADDED PAUSE
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{drivebase, 0.5, 1.889_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.889_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
      commands.push_back(
          std::make_unique<PauseRobot>(drivebase, 0.1_s));  // ADDED PAUSE
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveUntilPitchAngleChange{drivebase, 0.5}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing{drivebase}));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *MoveToDefenseAgainstScoringWall(Drivebase *drivebase,
                                                 std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                TurnDegreesImported{drivebase, 0.5, 90_deg},
                TurnDegreesImported(drivebase, 0.5, 180_deg),
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red2 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red1;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue2 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            DriveAtPowerForMeters{drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                  4_m},
            frc2::ConditionalCommand(
                DriveAtPowerForMeters{drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                      2_m},
                frc2::PrintCommand{"Doing nothing"},
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue2 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red2;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Red1 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                TurnDegreesImported{drivebase, 0.5, 90_deg},
                frc2::PrintCommand{"Doing nothing"},
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red2 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red1;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue2 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 2_m}));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, 25.8_deg},
            TurnDegreesImported{drivebase, 0.5, -25.8_deg}, [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue1 ||
                     teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue2 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 3.845_m}));

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *MoveToDefenseAgainstOuterWall(Drivebase *drivebase,
                                               std::string teamAndPosName) {
    const bool isBlue =
        teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
        AutonomousTeamAndStationPositions::Blue2 ||
        AutonomousTeamAndStationPositions::Blue3;
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                TurnDegreesImported{drivebase, 0.5, 90_deg},
                TurnDegreesImported(drivebase, 0.5, 180_deg),
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue1 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue2;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Red3 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Red2;
            })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            DriveAtPowerForMeters{drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                  4_m},
            frc2::ConditionalCommand(
                DriveAtPowerForMeters{drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                      2_m},
                frc2::PrintCommand{"Doing nothing"},
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue2 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Red2;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Red3 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue1;
            })));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                TurnDegreesImported{drivebase, 0.5, 90_deg},
                frc2::PrintCommand{"Doing nothing"},
                [teamAndPosName] {
                  return teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue1 ||
                         teamAndPosName ==
                             AutonomousTeamAndStationPositions::Blue2;
                }),
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Red1 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Red2;
            })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, 90_deg},
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            [isBlue] { return isBlue; })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -47.9_deg},
            TurnDegreesImported{drivebase, 0.5, 47.9_deg},
            [isBlue] { return isBlue; })));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.948_m}));

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreAndLeave(Drivebase *drivebase,
                               IntakeDeployment *intakeDeployment,
                               FloorEjection *floorEjection,
                               std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.0_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *JustCharge(Drivebase *drivebase, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveUntilPitchAngleChange(drivebase, -0.5)));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing(drivebase)));
    } else {
      const bool firstTurnIsCounterClockwise =
          (teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
           teamAndPosName == AutonomousTeamAndStationPositions::Red3);
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.719_m)));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveUntilPitchAngleChange(drivebase, 0.5)));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing(drivebase)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenCharge(Drivebase *drivebase,
                                 IntakeDeployment *intakeDeployment,
                                 FloorEjection *floorEjection,
                                 IntakeRoller *intakeRoller,
                                 std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenEndNearGamePieceCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      FloorEjection *floorEjection, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.1_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 5.0_m)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGamePieceThenGTFOCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      FloorEjection *floorEjection, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.0_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGamePieceThenChargeCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      FloorEjection *floorEjection, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(floorEjection)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreGTFOThenCharge(Drivebase *drivebase,
                                     IntakeDeployment *intakeDeployment,
                                     FloorEjection *floorEjection,
                                     std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(GTFODOCK(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreTwiceThenChargeCommand(Drivebase *drivebase,
                                             IntakeDeployment *intakeDeployment,
                                             IntakeRoller *intakeRoller,
                                             FloorEjection *floorEjection,
                                             std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(ScoreThenEndNearGamePieceCommand(
            drivebase, intakeDeployment, floorEjection, teamAndPosName)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        GamePiecePickupHelperCommand(drivebase, intakeRoller)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 5_m - 10_in)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
              drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }
    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

}  // namespace Helpers

frc2::Command *GetAutonomousCommand(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeRoller *intakeRoller,
                                    FloorEjection *floorEjection,
                                    std::string operationName,
                                    std::string teamAndPosName) {
  using namespace Helpers;

  if (operationName == AutonomousSelectedOperation::DoNothing) {
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } else if (operationName == AutonomousSelectedOperation::GTFO) {
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      static DriveAtPowerForMeters JustDriving{
          drivebase, -AutonomousSpeeds::DRIVE_SPEED,
          4.5_m - 10_in};  // TODO Change for all subseqeunt
                           // drives over the charging station
      return &JustDriving;
    } else {
      static DriveAtPowerForMeters JustDriving{
          drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.0_m};
      return &JustDriving;
    }
  } else if (operationName == AutonomousSelectedOperation::GTFODock) {
    return GTFODOCK(drivebase, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::MoveToDefenseAgainstScoringWall) {
    return MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::MoveToDefenseAgainstOuterWall) {
    return MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::ScoreAndLeave) {
    return ScoreAndLeave(drivebase, intakeDeployment, floorEjection,
                         teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::
                                  ScoreAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  ScoreAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        ScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(floorEjection)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(floorEjection)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::ScorePiece) {
    return ScoreGamePieceHelperCommand(floorEjection);
  } else if (operationName == AutonomousSelectedOperation::JustCharge) {
    return JustCharge(drivebase, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::ScoreThenCharge) {
    return ScoreThenCharge(drivebase, intakeDeployment, floorEjection,
                           intakeRoller, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreThenEndNearGamePiece) {
    return ScoreThenEndNearGamePieceCommand(drivebase, intakeDeployment,
                                            floorEjection, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropGamePiece) {
    return DropGamePieceHelperCommand(floorEjection);
  } else if (operationName == AutonomousSelectedOperation::DropAndGTFO) {
    return DropGamePieceThenGTFOCommand(drivebase, intakeDeployment,
                                        floorEjection, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropAndCharge) {
    return DropGamePieceThenChargeCommand(drivebase, intakeDeployment,
                                          floorEjection, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreTwiceThenCharge) {
    return ScoreTwiceThenChargeCommand(drivebase, intakeDeployment,
                                       intakeRoller, floorEjection,
                                       teamAndPosName);
  }

  static frc2::PrintCommand fallThroughCaseCommand(
      "*** Error: don't know what to do, based on "
      "selections!");
  return &fallThroughCaseCommand;  // CHANGE THIS
}

}  // namespace AutonomousCommands