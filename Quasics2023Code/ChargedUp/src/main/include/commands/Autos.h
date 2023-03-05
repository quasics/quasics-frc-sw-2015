#ifndef COMMANDS_AUTOS_H
#define COMMANDS_AUTOS_H

#include <frc2/command/Command.h>

#include "subsystems/Drivebase.h"
#include "subsystems/FloorEjection.h"
#include "subsystems/IntakeClamp.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"

namespace AutonomousCommands {
frc2::Command *GetAutonomousCommand(Drivebase *drivebase,
                                    IntakeDeployment *intakeDeployment,
                                    IntakeClamp *intakeClamp,
                                    FloorEjection *floorEjection,
                                    std::string operationName,
                                    std::string teamAndPosName);
}

#endif  // COMMANDS_AUTOS_H
