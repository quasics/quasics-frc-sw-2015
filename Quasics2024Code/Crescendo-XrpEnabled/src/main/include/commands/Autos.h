// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <string>

#include "subsystems/IDrivebase.h"

namespace AutonomousCommands {
  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase,
                                        std::string operationName,
                                        std::string teamAndPosName);

  namespace Helpers {
    frc2::CommandPtr backwardTest(IDrivebase &drivebase);

  }  // namespace Helpers
}  // namespace AutonomousCommands