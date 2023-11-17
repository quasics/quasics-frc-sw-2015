#pragma once

#include <string>

#include <frc2/command/Commands.h>

#include "subsystems/Drivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad, Drivebase* driveBase);

frc2::CommandPtr BuildTrajectory(frc::Pose2d startingPose, frc::Pose2d endingPose, bool driveForward, Drivebase* driveBase);

/*private:
void PerliminarySetup();*/
