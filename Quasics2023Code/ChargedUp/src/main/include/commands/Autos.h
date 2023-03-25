#ifndef COMMANDS_AUTOS_H
#define COMMANDS_AUTOS_H

#include <frc2/command/Command.h>

#include "subsystems/Drivebase.h"
#include "subsystems/FloorEjection.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"

namespace AutonomousCommands {
frc2::Command *GetAutonomousCommand(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeRoller *intakeRoller,
                                    FloorEjection *floorEjection,
                                    std::string operationName,
                                    std::string teamAndPosName);

//////////////////////////////////////////////////////////////
// Helper functions used in the primary command.  (These are
// exposed for testing support only.)
namespace Helpers {
  frc2::Command *DropGamePieceHelperCommand(IntakeDeployment *intakeDeployment,
                                            IntakeRoller *intakeRoller);

  frc2::Command *ClampScoreGamePieceHelperCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller);

  frc2::Command *ScoreGamePieceHelperCommand(
      FloorEjection *floorEjection);

  frc2::Command *GamePiecePickupHelperCommand(
      Drivebase *drivebase, IntakeRoller *intakeRoller,
      IntakeDeployment *intakeDeployment);

  frc2::Command *GTFODOCK(Drivebase *drivebase, std::string teamAndPosName);

  frc2::Command *MoveToDefenseAgainstScoringWall(Drivebase *drivebase,
                                                 std::string teamAndPosName);

  frc2::Command *MoveToDefenseAgainstOuterWall(Drivebase *drivebase,
                                               std::string teamAndPosName);

  frc2::Command *JustCharge(Drivebase *drivebase, std::string teamAndPosName);

  frc2::Command *GetScoreSequenceFromStartingPoint(Drivebase *drivebase,
                                                   FloorEjection *floorEjection,
                                                   IntakeRoller *intakeRoller);

  frc2::Command *ScoreThenCharge(Drivebase *drivebase,
                                 FloorEjection *floorEjection,
                                 IntakeRoller *intakeRoller,
                                 std::string teamAndPosName);

  frc2::Command *ScoreThenEndNearGamePieceCommand(Drivebase *drivebase,
                                                  FloorEjection floorEjection,
                                                  std::string teamAndPosName);

  frc2::Command *DropGamePieceThenGTFOCommand(Drivebase *drivebase,
                                              FloorEjection *floorEjection,
                                              std::string teamAndPosName);

  frc2::Command *DropGamePieceThenChargeCommand(
      Drivebase *drivebase, IntakeDeployment *intakeDeployment,
      IntakeRoller *intakeRoller, std::string teamAndPosName);

  frc2::Command *ScoreGTFOThenCharge(Drivebase *drivebase,
                                     FloorEjection *floorEjection,
                                     std::string teamAndPosName);

  frc2::Command *ScoreTwiceThenChargeCommand(Drivebase *drivebase,
                                             IntakeDeployment *intakeDeployment,
                                             IntakeRoller *intakeRoller,
                                             FloorEjection *floorEjection,
                                             std::string teamAndPosName);
}  // namespace Helpers
}  // namespace AutonomousCommands

#endif  // COMMANDS_AUTOS_H
