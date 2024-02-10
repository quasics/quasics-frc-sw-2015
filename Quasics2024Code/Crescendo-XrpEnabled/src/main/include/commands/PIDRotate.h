// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <networktables/GenericEntry.h>

#include "subsystems/IDrivebase.h"

#undef USE_DYNAMIC_DATA_FROM_DASHBOARD

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PIDRotate : public frc2::CommandHelper<frc2::Command, PIDRotate> {
 public:
  PIDRotate(IDrivebase& drivebase, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  void FeedForward();

 private:
  IDrivebase& m_drivebase;
  const units::degree_t m_angle;
  units::degree_t m_startingAngle = 0_deg;
  units::degree_t m_currentAngle = 0_deg;
  bool m_feedForward = true;
  bool m_activatePID = false;
  double m_rotationCorrection = 0;
  double m_speed = 0.5;
  double m_subtraction = 0;

  static constexpr double ANGLE_TOLERANCE = 1.0;
  static constexpr double VELOCITY_TOLERANCE = 0.0;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  std::unique_ptr<frc2::PIDController> dynamicPid;
  nt::GenericEntry* kP_entry = nullptr;
  nt::GenericEntry* kI_entry = nullptr;
  nt::GenericEntry* kD_entry = nullptr;
  nt::GenericEntry* angle_entry = nullptr;
  double angle = 0;
#else
  frc::PIDController m_pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
                           PIDTurningConstants::kD};
#endif
};
