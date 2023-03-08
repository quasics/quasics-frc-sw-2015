// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ConfigSettings.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Changes the value in config settings when a button is toggled in the
// controller allows for the intake to switch speeds between cone and cube
class SetCubeOrConeIntakeSpeed
    : public frc2::CommandHelper<frc2::CommandBase, SetCubeOrConeIntakeSpeed> {
 public:
  SetCubeOrConeIntakeSpeed(ConfigSettings* settings);

  void Initialize() override;

  bool IsFinished() override;

 private:
  ConfigSettings* m_settings;
};
