// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <networktables/GenericEntry.h>

#include "commands/PIDRotate.h"
#include "commands/TankDrive.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/Vision.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateToAprilTarget
    : public frc2::CommandHelper<frc2::Command, RotateToAprilTarget> {
 public:
  RotateToAprilTarget(IDrivebase& drivebase, Vision& vision, int ID);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Vision& m_vision;
  IDrivebase& m_drivebase;
  const int m_ID;
  bool m_activatePID = false;
  units::degrees_per_second_t m_rotationCorrection = 0_deg_per_s;
  units::degrees_per_second_t m_speed = 1_deg_per_s;

  static constexpr double ANGLE_TOLERANCE = 1.0;
  static constexpr double VELOCITY_TOLERANCE = 0.0;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  nt::GenericEntry* kP_entry = nullptr;
  nt::GenericEntry* kI_entry = nullptr;
  nt::GenericEntry* kD_entry = nullptr;
  nt::GenericEntry* angle_entry = nullptr;
#endif
  frc::PIDController m_pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
                           PIDTurningConstants::kD};
};
