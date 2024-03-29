// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/Lighting.h"
#include "subsystems/Vision.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  void ConfigureSimpleLightingExample();
  void ConfigureDirectionalLighting();

 private:
  frc::XboxController m_controller{0};

  Lighting m_lighting{LedConstants::LED_PWM_PORT, LedConstants::LED_STRIP_SIZE};
  std::unique_ptr<IDrivebase> m_drivebase;
#ifdef LEAK_VISION_TO_WORK_AROUND_CLEANUP_BUG
  // TODO: Clean this up, as the vision subclass will be leaked on
  // shutdown/restart. This is due to a crash bug someplace in the PhotonCamera
  // cleanup.
  Vision& m_vision = *(new Vision);
#else
  Vision m_vision;
#endif
};
