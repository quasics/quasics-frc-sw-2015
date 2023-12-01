// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/Lighting.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

 private:
  Lighting m_lighting{LedConstants::LED_PWM_PORT, LedConstants::LED_STRIP_SIZE};
  std::unique_ptr<IDrivebase> m_drivebase;
};
