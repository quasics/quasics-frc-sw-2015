#include "commands/Autos.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>
#include <string>

#include "Constants.h"
#include "commands/PauseRobot.h"
#include "commands/intake/AutoIntakeExtension.h"
#include "commands/intake/ExhaustWithRoller.h"
#include "commands/intake/ExhaustWithRollerAtSpeedForTime.h"
#include "commands/intake/ExtendIntakeAtSpeedForTime.h"
#include "commands/intake/IntakeWithRoller.h"
#include "commands/intake/IntakeWithRollerAtSpeedForTime.h"
#include "commands/intake/RetractIntakeAtSpeedForTime.h"
#include "commands/movement/DriveAtPowerForMeters.h"
#include "commands/movement/DriveUntilPitchAngleChange.h"
#include "commands/movement/PIDTurning.h"
#include "commands/movement/RotateAtAngle.h"
#include "commands/movement/SelfBalancing.h"
#include "commands/movement/StraightLineDriving.h"
#include "commands/movement/TurnDegreesImported.h"

#define USING_PID_TURNING

namespace AutonomousCommands {
namespace Helpers {
  //
  // Implementation of all of the helper functions
  //

  frc2::Command *IntakeDropGamePieceHelperCommand(
      IntakeDeployment *intakeDeployment, IntakeRoller *intakeRoller) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        new IntakeWithRollerAtSpeedForTime(intakeRoller, -0.2, 0.8_s)));
    /*commands.push_back(std::unique_ptr<frc2::Command>(
        new AutoIntakeExtension(intakeDeployment, 0.5)));*/
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *IntakeScoreGamePieceHelperCommand(
      IntakeDeployment *intakeDeployment, IntakeRoller *intakeRoller) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    /*commands.push_back(std::unique_ptr<frc2::Command>(
        new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.4_s)));*/
    commands.push_back(
        std::unique_ptr<frc2::Command>(new IntakeWithRollerAtSpeedForTime(
            intakeRoller, -IntakeConstants::RollerSpeeds::CONES, 1.5_s)));
    /*commands.push_back(std::unique_ptr<frc2::Command>(
        new AutoIntakeExtension(intakeDeployment, 0.5)));*/
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *MoveAndIntake(Drivebase *drivebase, IntakeRoller *intakeRoller,
                               units::meter_t distance) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
        drivebase, AutonomousSpeeds::DRIVE_SPEED, distance)));
    commands.push_back(std::unique_ptr<frc2::Command>(new IntakeWithRoller(
        intakeRoller, IntakeConstants::RollerSpeeds::CUBES)));

    return new frc2::ParallelRaceGroup(std::move(commands));
  }

  frc2::Command *GamePiecePickupHelperCommand(Drivebase *drivebase,
                                              IntakeRoller *intakeRoller) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(  // TODO fix this
        std::unique_ptr<frc2::Command>(
            MoveAndIntake(drivebase, intakeRoller, 1.3_m)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreTwoGamePiecesHelperCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, bool Dropping, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    if (Dropping) {
      commands.push_back(
          std::unique_ptr<frc2::Command>(DropThenEndNearGamePieceCommand(
              drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(ScoreThenEndNearGamePieceCommand(
              drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
    }
    commands.push_back(std::unique_ptr<frc2::Command>(
        DrivingBackToStartingPositionHelperCommand(
            drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
    if (Dropping) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    } else {
      commands.push_back(
          std::unique_ptr<frc2::Command>(ScoreThenEndNearGamePieceCommand(
              drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DrivingBackToStartingPositionHelperCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, -180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, -180_deg)));

#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 5_m - 10_in)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *moveToBlue3OrRed1(Drivebase *drivebase,
                                   std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, -90_deg},
            frc2::ConditionalCommand(
                PIDTurning{drivebase, 90_deg}, PIDTurning(drivebase, 180_deg),
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
#else
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
#endif

    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m},
            frc2::ConditionalCommand(
                StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED,
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
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, -90_deg},
            frc2::ConditionalCommand(
                PIDTurning{drivebase, 90_deg},
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
#else
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
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *moveToBlue1OrRed3(Drivebase *drivebase,
                                   std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, -90_deg},
            frc2::ConditionalCommand(
                PIDTurning{drivebase, 90_deg}, PIDTurning(drivebase, 180_deg),
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
#else
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
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m},
            frc2::ConditionalCommand(
                StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED,
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
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, -90_deg},
            frc2::ConditionalCommand(
                PIDTurning{drivebase, 90_deg},
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
#else
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
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *moveToDefenseFromBlue3OrRed1(Drivebase *drivebase,
                                              std::string teamAndPosName) {
    const bool isBlue =
        teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
        AutonomousTeamAndStationPositions::Blue2 ||
        AutonomousTeamAndStationPositions::Blue3;
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, 90_deg}, PIDTurning{drivebase, -90_deg},
            [isBlue] { return isBlue; })));
#else
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, 90_deg},
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            [isBlue] { return isBlue; })));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, -47.9_deg}, PIDTurning{drivebase, 47.9_deg},
            [isBlue] { return isBlue; })));
#else
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, -47.9_deg},
            TurnDegreesImported{drivebase, 0.5, 47.9_deg},
            [isBlue] { return isBlue; })));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.948_m}));

    return new frc2::SequentialCommandGroup(std::move(commands));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *moveToDefenseFromBlue1OrRed3(Drivebase *drivebase,
                                              std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 2_m}));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, 25.8_deg}, PIDTurning{drivebase, -25.8_deg},
            [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue1 ||
                     teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue2 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));
#else
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
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 3.845_m}));
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
              drivebase, -AutonomousSpeeds::OVER_CHARGING_STATION_SPEED,
              3.7_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
          drivebase, 0.75 * AutonomousSpeeds::OVER_CHARGING_STATION_SPEED,
          0.1_m}));
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
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
          // POSSIBLE ERROR
          drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.0_m}));

#ifdef USING_PID_TURNING
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              PIDTurning{drivebase, 90_deg}, PIDTurning{drivebase, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#else
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#endif
      commands.push_back(
          std::make_unique<PauseRobot>(drivebase, 0.3_s));  // ADDED PAUSE
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving{
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.889_m}));
#ifdef USING_PID_TURNING
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              PIDTurning{drivebase, -90_deg}, PIDTurning{drivebase, 90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#else
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#endif
      commands.push_back(
          std::make_unique<PauseRobot>(drivebase, 0.3_s));  // ADDED PAUSE
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
    commands.push_back(std::unique_ptr<frc2::Command>(
        moveToBlue1OrRed3(drivebase, teamAndPosName)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        moveToDefenseFromBlue1OrRed3(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *MoveToDefenseAgainstOuterWall(Drivebase *drivebase,
                                               std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        moveToBlue3OrRed1(drivebase, teamAndPosName)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        moveToDefenseFromBlue3OrRed1(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  /*
    frc2::Command *MoveToDefenseAgainstScoringWall(Drivebase *drivebase,
                                                   std::string teamAndPosName)
    { std::vector<std::unique_ptr<frc2::Command>> commands;

      // if red do this
      // if blue do this
      // blue 3 or red 1 will always be executed if for outer

      // turning for blue 1 and 2 will always be executed  SAME FOR RED 2 and
    3
      // Opposite Direction conditional command for distance to travel call
    blue 3
      // red 1 is blue 3 with

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
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Blue3;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED,
    4_m}, frc2::ConditionalCommand( StraightLineDriving{drivebase,
    AutonomousSpeeds::DRIVE_SPEED, 2_m}, frc2::PrintCommand{"Doing nothing"},
                  [teamAndPosName] {
                    return teamAndPosName ==
                               AutonomousTeamAndStationPositions::Blue2 ||
                           teamAndPosName ==
                               AutonomousTeamAndStationPositions::Red2;
                  }),
              [teamAndPosName] {
                return teamAndPosName ==
                           AutonomousTeamAndStationPositions::Red1 ||
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Blue3;
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
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Blue3;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new
    StraightLineDriving{ drivebase, AutonomousSpeeds::DRIVE_SPEED, 2_m}));

      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 25.8_deg},
              TurnDegreesImported{drivebase, 0.5, -25.8_deg}, [teamAndPosName]
    { return teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
                       teamAndPosName ==
                           AutonomousTeamAndStationPositions::Blue2 ||
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Blue3;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new
    StraightLineDriving{ drivebase, AutonomousSpeeds::DRIVE_SPEED, 3.845_m}));

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
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Red2;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              StraightLineDriving{drivebase, AutonomousSpeeds::DRIVE_SPEED,
    4_m}, frc2::ConditionalCommand( StraightLineDriving{drivebase,
    AutonomousSpeeds::DRIVE_SPEED, 2_m}, frc2::PrintCommand{"Doing nothing"},
                  [teamAndPosName] {
                    return teamAndPosName ==
                               AutonomousTeamAndStationPositions::Blue2 ||
                           teamAndPosName ==
                               AutonomousTeamAndStationPositions::Red2;
                  }),
              [teamAndPosName] {
                return teamAndPosName ==
                           AutonomousTeamAndStationPositions::Red3 ||
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Blue1;
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
                       teamAndPosName ==
    AutonomousTeamAndStationPositions::Red2;
              })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new
    StraightLineDriving{ drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));

      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [isBlue] { return isBlue; })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new
    StraightLineDriving{ drivebase, AutonomousSpeeds::DRIVE_SPEED, 4_m}));

      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, -47.9_deg},
              TurnDegreesImported{drivebase, 0.5, 47.9_deg},
              [isBlue] { return isBlue; })));
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new
    StraightLineDriving{ drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.948_m}));

      return new frc2::SequentialCommandGroup(std::move(commands));
    }
  */

  frc2::Command *ScoreAndLeave(Drivebase *drivebase,
                               IntakeDeployment *intakeDeployment,
                               IntakeRoller *intakeRoller,
                               std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(  // POSSIBLE ERROR
              drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(  // POSSIBLE ERROR
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
#ifdef USING_PID_TURNING
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              PIDTurning{drivebase, 90_deg},  // CHANGED
              PIDTurning{drivebase, -90_deg}, [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#else
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 95_deg},  // CHANGED
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#endif
      commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 1.719_m)));
#ifdef USING_PID_TURNING
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              PIDTurning{drivebase, 90_deg}, PIDTurning{drivebase, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#else
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              TurnDegreesImported{drivebase, 0.5, 90_deg},
              TurnDegreesImported{drivebase, 0.5, -90_deg},
              [firstTurnIsCounterClockwise]() {
                return firstTurnIsCounterClockwise;
              })));
#endif
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveUntilPitchAngleChange(drivebase, 0.5)));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing(drivebase)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenCharge(Drivebase *drivebase,
                                 IntakeDeployment *intakeDeployment,
                                 IntakeRoller *intakeRoller,
                                 std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenEndNearGamePieceCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, 180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 6_m)));
      /*commands.push_back(std::unique_ptr<frc2::Command>(new
         StraightLineDriving( drivebase,
         AutonomousSpeeds::DRIVE_SPEED, 4.7_m)));*/
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 4.8_m)));
      /*commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                  3.5_m)));  // URGENT CHANGED THIS*/
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropThenEndNearGamePieceCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));

#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, 180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
#endif
    commands.push_back(std::unique_ptr<frc2::Command>(new AutoIntakeExtension(
        intakeDeployment, AutonomousSpeeds::INTAKE_EXTENSION_SPEED)));
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      /*commands.push_back(std::unique_ptr<frc2::Command>(newf
         StraightLineDriving( drivebase,
         AutonomousSpeeds::DRIVE_SPEED, 4.7_m)));*/
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 6_m)));

    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 4.8_m)));
      /*commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(drivebase, AutonomousSpeeds::DRIVE_SPEED,
                                  3.5_m)));  // URGENT CHANGED THIS*/
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }
  frc2::Command *DropGamePieceThenGTFOCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));

    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(  // POSSIBLE ERROR
              drivebase, -1 * AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new StraightLineDriving(  // POSSIBLE ERROR
              drivebase, -1 * AutonomousSpeeds::DRIVE_SPEED, 4.0_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGamePieceThenChargeCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreGTFOThenCharge(Drivebase *drivebase,
                                     IntakeDeployment *intakeDeployment,
                                     IntakeRoller *intakeRoller,
                                     std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));

    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, -AutonomousSpeeds::DRIVE_SPEED, 0.25_m}));  // POSSIBLE ERROR
    commands.push_back(
        std::unique_ptr<frc2::Command>(GTFODOCK(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGTFOThenCharge(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeRoller *intakeRoller,
                                    std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));

    commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{
        drivebase, -AutonomousSpeeds::DRIVE_SPEED, 0.25_m}));
    commands.push_back(
        std::unique_ptr<frc2::Command>(GTFODOCK(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreTwiceThenChargeCommand(Drivebase *drivebase,
                                             IntakeDeployment *intakeDeployment,
                                             IntakeRoller *intakeRoller,
                                             std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(ScoreThenEndNearGamePieceCommand(
            drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
/*commands.push_back(
    std::unique_ptr<frc2::Command>(MoveAndIntake(drivebase,
   intakeRoller)));*/
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, -180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, -180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 5_m - 10_in)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropTwiceThenChargeCommand(Drivebase *drivebase,
                                            IntakeDeployment *intakeDeployment,
                                            IntakeRoller *intakeRoller,
                                            std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(DropThenEndNearGamePieceCommand(
            drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
/*commands.push_back(
    std::unique_ptr<frc2::Command>(MoveAndIntake(drivebase,
   intakeRoller)));*/
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, -180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, -180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 5_m - 10_in)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropTwo(Drivebase *drivebase,
                         IntakeDeployment *intakeDeployment,
                         IntakeRoller *intakeRoller,
                         std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(DropThenEndNearGamePieceCommand(
            drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
/*commands.push_back(
    std::unique_ptr<frc2::Command>(MoveAndIntake(drivebase,
   intakeRoller)));*/
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, -180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, -180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 5_m - 10_in)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m)));
    }
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, -30_deg)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new IntakeWithRollerAtSpeedForTime(intakeRoller, -0.8, 0.75_s)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  // URGENT ON THE RED SIDE FOR RED 2 SET UP CLOSER TO LOADING STATION
  // ON THE BLUE SIDE SET UP TO COLLECT THE CUBE FURTHER AWAY FROM THE STATION
  frc2::Command *DropThree(Drivebase *drivebase,
                           IntakeDeployment *intakeDeployment,
                           IntakeRoller *intakeRoller,
                           std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        DropTwo(drivebase, intakeDeployment, intakeRoller, teamAndPosName)));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, 180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 5_m)));

    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          MoveAndIntake(drivebase, intakeRoller, 3.8_m)));
    }
    const bool turnLeftForThirdCube =
        ((teamAndPosName == AutonomousTeamAndStationPositions::Red3 ||
          teamAndPosName == AutonomousTeamAndStationPositions::Red2) ||
         (teamAndPosName == AutonomousTeamAndStationPositions::Blue3 ||
          teamAndPosName == AutonomousTeamAndStationPositions::Blue2));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, 90_deg},  // CHANGED
            PIDTurning{drivebase, -90_deg},
            [turnLeftForThirdCube]() { return turnLeftForThirdCube; })));
#else
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, 90_deg},  // CHANGED
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            [turnLeftForThirdCube]() { return turnLeftForThirdCube; })));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveAndIntake(drivebase, intakeRoller, 1_m)));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new PIDTurning(drivebase, 180_deg)));
#else
    commands.push_back(std::unique_ptr<frc2::Command>(
        new TurnDegreesImported(drivebase, 0.5, 180_deg)));
#endif
    commands.push_back(std::make_unique<PauseRobot>(drivebase, 0.3_s));
    commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
        drivebase, AutonomousSpeeds::DRIVE_SPEED, 1_m)));
#ifdef USING_PID_TURNING
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            PIDTurning{drivebase, 90_deg},  // CHANGED
            PIDTurning{drivebase, -90_deg},
            [turnLeftForThirdCube]() { return !turnLeftForThirdCube; })));
#else
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            TurnDegreesImported{drivebase, 0.5, 90_deg},  // CHANGED
            TurnDegreesImported{drivebase, 0.5, -90_deg},
            [turnLeftForThirdCube]() { return !turnLeftForThirdCube; })));
#endif  // negated to reverse the turn
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.5_m - 10_in)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(new StraightLineDriving(
          drivebase, AutonomousSpeeds::DRIVE_SPEED, 4.0_m)));
    }
    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));

    // if at red 3,2 or blue 3, 2 make a left
    // if at blue 1 or red 1 make a right

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

}  // namespace Helpers
frc2::Command *GetAutonomousCommand(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeRoller *intakeRoller,
                                    std::string operationName,
                                    std::string teamAndPosName) {
  using namespace Helpers;

  if (operationName == AutonomousSelectedOperation::DoNothing) {
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } else if (operationName == AutonomousSelectedOperation::GTFO) {
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      static StraightLineDriving JustDriving{
          drivebase, -AutonomousSpeeds::DRIVE_SPEED,  // POSSIBLE ERROR
          4.5_m - 10_in};  // TODO Change for all subseqeunt
                           // drives over the charging station
      return &JustDriving;
    } else {
      static StraightLineDriving JustDriving{
          drivebase, -AutonomousSpeeds::DRIVE_SPEED, 4.0_m};  // POSSIBLE ERROR
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
    return ScoreAndLeave(drivebase, intakeDeployment, intakeRoller,
                         teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::
                                  ScoreAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  ScoreAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeScoreGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::ScorePiece) {
    return IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller);
  } else if (operationName == AutonomousSelectedOperation::JustCharge) {
    return JustCharge(drivebase, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::ScoreThenCharge) {
    return ScoreThenCharge(drivebase, intakeDeployment, intakeRoller,
                           teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreThenEndNearGamePiece) {
    return ScoreThenEndNearGamePieceCommand(drivebase, intakeDeployment,
                                            intakeRoller, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropGamePiece) {
    return IntakeDropGamePieceHelperCommand(intakeDeployment, intakeRoller);
  } else if (operationName == AutonomousSelectedOperation::DropAndGTFO) {
    return DropGamePieceThenGTFOCommand(drivebase, intakeDeployment,
                                        intakeRoller, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropAndCharge) {
    return DropGamePieceThenChargeCommand(drivebase, intakeDeployment,
                                          intakeRoller, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropGTFOCharge) {
    return DropGTFOThenCharge(drivebase, intakeDeployment, intakeRoller,
                              teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::ScoreGTFOCharge) {
    return ScoreGTFOThenCharge(drivebase, intakeDeployment, intakeRoller,
                               teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreTwiceThenCharge) {
    return ScoreTwiceThenChargeCommand(drivebase, intakeDeployment,
                                       intakeRoller, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::DropTwiceThenCharge) {
    return DropTwiceThenChargeCommand(drivebase, intakeDeployment, intakeRoller,
                                      teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropTwice) {
    return DropTwo(drivebase, intakeDeployment, intakeRoller, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropThree) {
    return DropThree(drivebase, intakeDeployment, intakeRoller, teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::DropThenEndNearGamePiece) {
    return DropThenEndNearGamePieceCommand(drivebase, intakeDeployment,
                                           intakeRoller, teamAndPosName);
  }

  static frc2::PrintCommand fallThroughCaseCommand(
      "*** Error: don't know what to do, based on "
      "selections!");
  return &fallThroughCaseCommand;  // CHANGE THIS
}

}  // namespace AutonomousCommands