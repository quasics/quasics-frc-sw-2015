// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <networktables/GenericEntry.h>

#include "subsystems/IDrivebase.h"

#undef USE_DYNAMIC_DATA_FROM_DASHBOARD

// TODO: (CODE_REVIEW) Add comments.
class PIDRotate : public frc2::CommandHelper<frc2::Command, PIDRotate> {
 public:
  PIDRotate(IDrivebase& drivebase, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IDrivebase& m_drivebase;
  units::degree_t m_targetAngle;
  units::degree_t m_currentAngle = 0_deg;
  bool m_activatePID = false;
  units::degrees_per_second_t m_rotationCorrection = 0_deg_per_s;
  units::degrees_per_second_t m_speed = 30_deg_per_s;

  static constexpr double ANGLE_TOLERANCE = 1.0;
  static constexpr double VELOCITY_TOLERANCE = 0.0;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  nt::GenericEntry* kP_entry = nullptr;
  nt::GenericEntry* kI_entry = nullptr;
  nt::GenericEntry* kD_entry = nullptr;
  nt::GenericEntry* angle_entry = nullptr;
#endif
  frc::PIDController m_pid;
};
