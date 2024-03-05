#pragma once

#include <frc2/command/Commands.h>

#include <string>

#include "subsystems/IDrivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad,
                                         IDrivebase &driveBase);
frc::Pose2d GetTrajectoryInitialPose(std::string fileToLoad);
frc::Pose2d GetTrajectoryFinalPose(std::string fileToLoad);