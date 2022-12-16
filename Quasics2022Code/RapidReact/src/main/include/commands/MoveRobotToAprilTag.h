// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"
#include "subsystems/PhotonVision.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveRobotToAprilTag
    : public frc2::CommandHelper<frc2::CommandBase, MoveRobotToAprilTag> {
 public:
  MoveRobotToAprilTag(Drivebase* drivebase, PhotonVision* photonVision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc2::PIDController controller{.1, 0, 0};

  double rotationSpeed;

  PhotonVision* m_photonvision;
  Drivebase* m_drivebase;
};
