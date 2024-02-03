// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <string>

#include "subsystems/IDrivebase.h"
#include "subsystems/Shooter.h"

namespace AutonomousCommands {
  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase,
                                        Shooter &m_shooter,
                                        std::string operationName,
                                        std::string teamAndPosName,
                                        std::string score2Dest,
                                        std::string score3Dest);

  namespace Helpers {
    frc2::CommandPtr backwardTest(IDrivebase &drivebase);
    frc2::CommandPtr blue1aAmp1AmpGo(IDrivebase &drivebase);
    frc2::CommandPtr resetOdometryToStartingPosition(IDrivebase &drivebase,
                                                     std::string position);

  }  // namespace Helpers
}  // namespace AutonomousCommands