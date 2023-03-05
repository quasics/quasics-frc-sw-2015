// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Drivebase.h>

#include "Constants.h"
#include "subsystems/PhotonLibVision.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * CODE_REVIEW(matthew): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.
 */
class AprilTagDriveToTarget
    : public frc2::CommandHelper<frc2::CommandBase, AprilTagDriveToTarget> {
 public:
  AprilTagDriveToTarget(PhotonLibVision* photonLibVision, Drivebase* drivebase,
                        int targetToDriveTo);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  void UpdateDrivingParameters();

 private:
  frc2::PIDController forwardController{PhotonVisionConstants::LinearPID::kP,
                                        PhotonVisionConstants::LinearPID::kI,
                                        PhotonVisionConstants::LinearPID::kD};
  frc2::PIDController turnController{PhotonVisionConstants::AngularPID::kP,
                                     PhotonVisionConstants::AngularPID::kI,
                                     PhotonVisionConstants::AngularPID::kD};

  PhotonLibVision* m_photonLibVision;
  Drivebase* m_drivebase;
  const int m_targetToDriveTo;
  units::degree_t m_angle;
  units::meter_t m_distance;
};
