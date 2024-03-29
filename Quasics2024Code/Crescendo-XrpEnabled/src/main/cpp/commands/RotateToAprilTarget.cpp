// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateToAprilTarget.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>
#include <frc2/command/PIDCommand.h>
#include <units/math.h>

#include <cmath>
#include <iostream>

#include "Constants.h"

// This uses a copy of the PID Rotate command
// TODO: Talk to Mr. Healy about a better way to share code between the
// PIDRotate command and this one.
RotateToAprilTarget::RotateToAprilTarget(IDrivebase& drivebase, Vision& vision,
                                         int ID)
    : m_drivebase(drivebase),
      m_ID(ID),
      m_vision(vision),
      m_pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
            PIDTurningConstants::kD} {
  AddRequirements({&m_drivebase, &m_vision});
}

// Called when the command is initially scheduled.
void RotateToAprilTarget::Initialize() {
  m_pid.Reset();
  m_pid.SetTolerance(ANGLE_TOLERANCE, VELOCITY_TOLERANCE);
  m_pid.EnableContinuousInput(-180, 180);
}

// Called repeatedly when this Command is scheduled to run
void RotateToAprilTarget::Execute() {
  auto possibleTarget = m_vision.GetIdentifiedAprilTarget(m_ID);
  if (!possibleTarget.has_value()) {
    return;
  }
  auto target = possibleTarget.value();
  units::degrees_per_second_t rotationCorrection =
      PIDTurningConstants::PID_multiplier *
      (m_pid.Calculate(-target.GetYaw(), 0));

  if (rotationCorrection >= 0_deg_per_s) {
    m_drivebase.arcadeDrive(0_mps, rotationCorrection + 2_deg_per_s);
  } else {
    m_drivebase.arcadeDrive(0_mps, rotationCorrection - 2_deg_per_s);
  }
}

// Called once the command ends or is interrupted.
void RotateToAprilTarget::End(bool interrupted) {
  m_drivebase.tankDrive(0, 0);
}

// Returns true when the command should end.
bool RotateToAprilTarget::IsFinished() {
  if (!m_vision.AprilTagTargetIdentified(m_ID)) {
    return true;
  }
  auto possibleTarget = m_vision.GetIdentifiedAprilTarget(m_ID);
  if (!possibleTarget.has_value()) {
    return true;
  }
  auto target = possibleTarget.value();
  if (std::abs(target.GetYaw()) < .5 &&
      units::math::abs(m_drivebase.getWheelSpeeds().left) < 0.3_mps &&
      units::math::abs(m_drivebase.getWheelSpeeds().right) < 0.3_mps) {
    return true;
  }
  return false;
}
