#include "commands/Autos.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>
#include <string>

#include "Constants.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/DriveUntilPitchAngleChange.h"
#include "commands/ExtendIntakeAtSpeedForTime.h"
#include "commands/MoveFloorEjectionAtPowerForTime.h"
#include "commands/ReleaseWithIntakeAtSpeedForTime.h"
#include "commands/RetractIntakeAtSpeedForTime.h"
#include "commands/RotateAtAngle.h"
#include "commands/SelfBalancing.h"

namespace AutonomousCommands {
// The functions that go into the autonomous chooser thingy above
namespace {  // anonymous namespace
  //
  // Prototypes for all of the helper functions
  //
  frc2::SequentialCommandGroup *DropGamePieceHelperCommand(
      IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp);

  frc2::SequentialCommandGroup *ClampScoreGamePieceHelperCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp);

  frc2::SequentialCommandGroup *RollerScoreGamePieceHelperCommand(
      FloorEjection *floorEjection);

  frc2::Command *GTFODOCK(Drivebase *drivebase, std::string teamAndPosName);

  frc2::Command *MoveToDefenseAgainstScoringWall(Drivebase *drivebase,
                                                 std::string teamAndPosName);

  frc2::Command *MoveToDefenseAgainstOuterWall(Drivebase *drivebase,
                                               std::string teamAndPosName);

  frc2::Command *JustCharge(Drivebase *drivebase, std::string teamAndPosName);

  frc2::SequentialCommandGroup *GetScoreSequenceFromStartingPoint(
      Drivebase *drivebase, FloorEjection *floorEjection,
      IntakeClamp *intakeClamp);

  frc2::Command *ScoreThenCharge(Drivebase *drivebase,
                                 FloorEjection *floorEjection,
                                 IntakeClamp *intakeClamp,
                                 std::string teamAndPosName);

  frc2::Command *ScoreThenEndNearGamePieceCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName);

  frc2::Command *DropGamePieceThenGTFOCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName);

  frc2::Command *DropGamePieceThenChargeCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName);

  frc2::Command *ScoreGTFOThenCharge(Drivebase *drivebase,
                                     FloorEjection *floorEjection,
                                     std::string teamAndPosName);

  //
  // Implementation of all of the helper functions
  //
  frc2::SequentialCommandGroup *DropGamePieceHelperCommand(
      IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ExtendIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ReleaseWithIntakeAtSpeedForTime(intakeClamp, 0.5, 0.3_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::SequentialCommandGroup *ClampScoreGamePieceHelperCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ExtendIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 0.3_m)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ReleaseWithIntakeAtSpeedForTime(intakeClamp, 0.5, 0.3_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, -0.5, 0.3_m)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::SequentialCommandGroup *RollerScoreGamePieceHelperCommand(
      FloorEjection *floorEjection) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        new MoveFloorEjectionAtPowerForTime(floorEjection, 0.5, 0.3_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new MoveFloorEjectionAtPowerForTime(floorEjection, 0.3, 0.5_s)));
    /*
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ExtendIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 0.3_m)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new ExhaustWithRollerAtSpeedForTime(intakeRoller, 0.5, 0.3_s)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, -0.5, 0.3_m)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
      */
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *GTFODOCK(Drivebase *drivebase, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      // In this case, we need to move back out of the community area (for the
      // mobility points), and then move forward and balance on the charging
      // station.
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{drivebase, -0.5, 4.5_m}));
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
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{drivebase, -0.5, 4.0_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, 90_deg},
                                       RotateAtAngle{drivebase, 0.5, -90_deg},
                                       [firstTurnIsCounterClockwise]() {
                                         return firstTurnIsCounterClockwise;
                                       })));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{drivebase, 0.5, 1.889_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, -90_deg},
                                       RotateAtAngle{drivebase, 0.5, 90_deg},
                                       [firstTurnIsCounterClockwise]() {
                                         return firstTurnIsCounterClockwise;
                                       })));
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
            RotateAtAngle{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                RotateAtAngle{drivebase, 0.5, 90_deg},
                RotateAtAngle(drivebase, 0.5, 180_deg),
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

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            DriveAtPowerForMeters{drivebase, 0.5, 4_m},
            frc2::ConditionalCommand(
                DriveAtPowerForMeters{drivebase, 0.5, 2_m},
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
            RotateAtAngle{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                RotateAtAngle{drivebase, 0.5, 90_deg},
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

    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{drivebase, 0.5, 2_m}));

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            RotateAtAngle{drivebase, 0.5, 25.8_deg},
            RotateAtAngle{drivebase, 0.5, -25.8_deg}, [teamAndPosName] {
              return teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue1 ||
                     teamAndPosName ==
                         AutonomousTeamAndStationPositions::Blue2 ||
                     teamAndPosName == AutonomousTeamAndStationPositions::Blue3;
            })));

    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{drivebase, 0.5, 3.845_m}));

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
            RotateAtAngle{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                RotateAtAngle{drivebase, 0.5, 90_deg},
                RotateAtAngle(drivebase, 0.5, 180_deg),
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

    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            DriveAtPowerForMeters{drivebase, 0.5, 4_m},
            frc2::ConditionalCommand(
                DriveAtPowerForMeters{drivebase, 0.5, 2_m},
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
            RotateAtAngle{drivebase, 0.5, -90_deg},
            frc2::ConditionalCommand(
                RotateAtAngle{drivebase, 0.5, 90_deg},
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

    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{drivebase, 0.5, 4_m}));

    commands.push_back(std::unique_ptr<frc2::Command>(
        new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, 90_deg},
                                     RotateAtAngle{drivebase, 0.5, -90_deg},
                                     [isBlue] { return isBlue; })));

    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{drivebase, 0.5, 4_m}));

    commands.push_back(std::unique_ptr<frc2::Command>(
        new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, -47.9_deg},
                                     RotateAtAngle{drivebase, 0.5, 47.9_deg},
                                     [isBlue] { return isBlue; })));

    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{drivebase, 0.5, 1.948_m}));

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  /*
  frc2::Command *ScoreAndLeave(std::string teamAndPosName) {
                                               }*/

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
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, 90_deg},
                                       RotateAtAngle{drivebase, 0.5, -90_deg},
                                       [firstTurnIsCounterClockwise]() {
                                         return firstTurnIsCounterClockwise;
                                       })));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters(drivebase, 0.5, 1.719_m)));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{drivebase, 0.5, 90_deg},
                                       RotateAtAngle{drivebase, 0.5, -90_deg},
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

  frc2::SequentialCommandGroup *GetScoreSequenceFromStartingPoint(
      Drivebase *drivebase, FloorEjection *floorEjection,
      IntakeClamp *intakeClamp) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
#ifdef USING_ROLLER_FOR_AUTO_INTAKE
    commands.push_back(std::unique_ptr<frc2::Command>(
        RollerScoreGamePieceHelperCommand(floorEjection)));
    std::cerr << "Using roller for auto intake.\n";
#elif defined(USING_CLAMP_FOR_AUTO_INTAKE)
    commands.push_back(std::unique_ptr<frc2::Command>(
        ClampScoreGamePieceHelperCommand(intakeClamp)));
    std::cerr << "Using clamp for auto intake.\n";
#else
    // Paranoid fallback: at least say you couldn't do it
    commands.push_back(std::unique_ptr<frc2::Command>(
        new frc2::PrintCommand("Don't know how to deposit game piece!")));
    std::cerr << "********** Don't know how to deposit game piece!\n";
#endif

    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenCharge(Drivebase *drivebase,
                                 FloorEjection *floorEjection,
                                 IntakeClamp *intakeClamp,
                                 std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(GetScoreSequenceFromStartingPoint(
            drivebase, floorEjection, intakeClamp)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreThenEndNearGamePieceCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::unique_ptr<frc2::Command>(ClampScoreGamePieceHelperCommand(
            drivebase, intakeDeployment, intakeClamp)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle(drivebase, 0.5, 180_deg)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters(drivebase, 0.5, 5.0_m)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters(drivebase, 0.5, 4.5_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGamePieceThenGTFOCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters(drivebase, 0.5, 4.5_m)));
    } else {
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters(drivebase, 0.5, 4.0_m)));
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *DropGamePieceThenChargeCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeClamp *intakeClamp, std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(JustCharge(drivebase, teamAndPosName)));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  frc2::Command *ScoreGTFOThenCharge(Drivebase *drivebase,
                                     FloorEjection *floorEjection,
                                     std::string teamAndPosName) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    return new frc2::SequentialCommandGroup(std::move(commands));
    commands.push_back(std::unique_ptr<frc2::Command>(
        RollerScoreGamePieceHelperCommand(floorEjection)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(GTFODOCK(drivebase, teamAndPosName)));
  }

}  // anonymous namespace

frc2::Command *GetAutonomousCommand(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeClamp *intakeClamp,
                                    FloorEjection *floorEjection,
                                    std::string operationName,
                                    std::string teamAndPosName) {
  // frc2::Command *selectedOperation =
  //     m_RobotSequenceAutonomousOptions.GetSelected();
  // frc2::Command *teamAndPosCmd =
  //     m_TeamAndStationAutonomousOptions.GetSelected();
  // if (selectedOperation == nullptr || teamAndPosCmd == nullptr) {
  //   // This shouldn't happen if things were set up right.  But it did.  So
  //   they
  //   // weren't. We'll bail out, but at least return a valid pointer that will
  //   // tell us something went wrong when it's run.
  //   static frc2::PrintCommand somethingIsScrewyCommand(
  //       "Selection error: can't decide what to do");
  //   return &somethingIsScrewyCommand;
  // }

  if (operationName == AutonomousSelectedOperation::DoNothing) {
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } else if (operationName == AutonomousSelectedOperation::GTFO) {
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      static DriveAtPowerForMeters JustDriving{
          drivebase, -0.5, 4.5_m - 10_in};  // TODO Change for all subseqeunt
                                            // drives over the charging station
      return &JustDriving;
    } else {
      static DriveAtPowerForMeters JustDriving{drivebase, -0.5, 4.0_m};
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
  } /*else if (operationName == AutonomousSelectedOperation::ScoreAndLeave) {
    return ScoreAndLeave(teamAndPosName);
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } */
  else if (operationName == AutonomousSelectedOperation::
                                ScoreAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(
        std::unique_ptr<frc2::Command>(ClampScoreGamePieceHelperCommand(
            drivebase, intakeDeployment, intakeClamp)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  ScoreAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(
        std::unique_ptr<frc2::Command>(ClampScoreGamePieceHelperCommand(
            drivebase, intakeDeployment, intakeClamp)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstScoringWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstScoringWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::
                                  DropAndMoveToDefenseAgainstOuterWall) {
    std::vector<std::unique_ptr<frc2::Command>> commands;

    commands.push_back(std::unique_ptr<frc2::Command>(
        DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
    commands.push_back(std::unique_ptr<frc2::Command>(
        MoveToDefenseAgainstOuterWall(drivebase, teamAndPosName)));

    return new frc2::SequentialCommandGroup(std::move(commands));

  } else if (operationName == AutonomousSelectedOperation::ScorePiece) {
    return ClampScoreGamePieceHelperCommand(drivebase, intakeDeployment,
                                            intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::JustCharge) {
    return JustCharge(drivebase, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::ScoreThenCharge) {
    return ScoreThenCharge(drivebase, floorEjection, intakeClamp,
                           teamAndPosName);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreThenEndNearGamePiece) {
    return ScoreThenEndNearGamePieceCommand(drivebase, intakeDeployment,
                                            intakeClamp, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropGamePiece) {
    return DropGamePieceHelperCommand(intakeDeployment, intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::DropAndGTFO) {
    return DropGamePieceThenGTFOCommand(drivebase, intakeDeployment,
                                        intakeClamp, teamAndPosName);
  } else if (operationName == AutonomousSelectedOperation::DropAndCharge) {
    return DropGamePieceThenChargeCommand(drivebase, intakeDeployment,
                                          intakeClamp, teamAndPosName);
  }

  static frc2::PrintCommand fallThroughCaseCommand(
      "*** Error: don't know what to do, based on "
      "selections!");
  return &fallThroughCaseCommand;  // CHANGE THIS
}

}  // namespace AutonomousCommands
