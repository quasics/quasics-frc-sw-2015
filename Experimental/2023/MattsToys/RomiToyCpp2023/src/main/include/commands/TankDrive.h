// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(Drivetrain* driveTrain,
                    std::function<double()> leftSpeedSupplier,
                    std::function<double()> rightRotateSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Drivetrain* m_driveTrain;
  std::function<double()> m_leftSpeedSupplier;
  std::function<double()> m_rightSpeedSupplier;
};
