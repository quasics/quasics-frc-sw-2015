// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IDrivebase.h"

// TODO: (CODE_REVIEW) Add comments.
class SetRobotOdometry
    : public frc2::CommandHelper<frc2::Command, SetRobotOdometry> {
 public:
  SetRobotOdometry(IDrivebase &driveBase, frc::Pose2d pose);

  void Initialize() override;

  bool IsFinished() override;

 private:
  IDrivebase &m_driveBase;
  frc::Pose2d m_pose;
};
