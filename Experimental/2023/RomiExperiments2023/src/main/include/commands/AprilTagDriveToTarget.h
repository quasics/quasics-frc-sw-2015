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
 */

// Given the Parameters and the assigned constants in the Constants.h file under
// PhotonVisionConstants This command will drive to the designated target and
// stop at a designated distance in Constants file
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
