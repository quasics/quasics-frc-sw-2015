// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "CommonDriveSubsystem.h"

/**
 * A sample implementation of an "curvature drive" command, for use in teleop
 * mode.
 */
class TeleopCurvatureDrive
    : public frc2::CommandHelper<frc2::CommandBase, TeleopCurvatureDrive> {
 public:
  TeleopCurvatureDrive(CommonDriveSubsystem* subsystem,
                       std::function<double()> xAxisSpeedSupplier,
                       std::function<double()> zAxisRotateSupplier,
                       std::function<bool()> isQuickTurnSupplier)
      : m_drive{subsystem},
        m_xaxisSpeedSupplier{xAxisSpeedSupplier},
        m_zaxisRotateSupplier{zAxisRotateSupplier},
        m_isQuickTurnSupplier{isQuickTurnSupplier} {
    AddRequirements({subsystem});
  }

  //
  // Methods from the "Command" class.
 public:
  void Execute() override {
    m_drive->CurvatureDrive(m_xaxisSpeedSupplier(), m_zaxisRotateSupplier(), m_isQuickTurnSupplier());
  }

  void End(bool interrupted) override {
    m_drive->Stop();
  }

 private:
  CommonDriveSubsystem* m_drive;
  std::function<double()> m_xaxisSpeedSupplier;
  std::function<double()> m_zaxisRotateSupplier;
  std::function<bool()> m_isQuickTurnSupplier;
};
