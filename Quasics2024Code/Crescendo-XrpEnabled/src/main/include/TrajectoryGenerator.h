#pragma once

#include <frc2/command/Commands.h>

#include <string>

// Forward declaration, to try to improve compile-time.
class IDrivebase;

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad,
                                         IDrivebase &driveBase);
frc::Pose2d GetTrajectoryInitialPose(std::string fileToLoad);
frc::Pose2d GetTrajectoryFinalPose(std::string fileToLoad);