// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/GenericEntry.h>

#include "subsystems/Drivebase.h"

#undef USE_DYNAMIC_DATA_FROM_DASHBOARD

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PIDTurning : public frc2::CommandHelper<frc2::CommandBase, PIDTurning> {
 public:
  PIDTurning(Drivebase* drivebase, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  void FeedForward();

 private:
  Drivebase* m_drivebase;
  const units::degree_t m_angle;
  units::degree_t startingAngle = 0_deg;
  units::degree_t currentAngle = 0_deg;
  bool feedForward = true;
  bool activatePID = false;
  double rotationCorrection = 0;
  double m_speed = 0.5;
  double m_subtraction = 0;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  std::unique_ptr<frc2::PIDController> dynamicPid;
  nt::GenericEntry* kP_entry = nullptr;
  nt::GenericEntry* kI_entry = nullptr;
  nt::GenericEntry* kD_entry = nullptr;
  nt::GenericEntry* angle_entry = nullptr;
  double angle = 0;
#else
  frc2::PIDController pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
                          PIDTurningConstants::kD};
#endif
};
