#pragma once

#include <frc2/command/Commands.h>

#include <string>

#include "subsystems/IDrivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad,
                                         IDrivebase* driveBase, bool reversed);
frc::Pose2d GetTrajectoryInitialPose(std::string fileToLoad);