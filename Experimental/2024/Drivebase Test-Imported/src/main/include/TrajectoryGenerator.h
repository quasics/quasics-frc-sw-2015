#pragma once

#include <string>

#include <frc2/command/Commands.h>

#include "subsystems/Drivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad, Drivebase* driveBase);
