// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateToAprilTarget.h"

#include <cmath>

RotateToAprilTarget::RotateToAprilTarget(IDrivebase& drivebase, Vision& vision,
                                         int ID)
    : m_drivebase(drivebase), m_ID(ID), m_vision(vision) {
  AddRequirements(&m_drivebase);
  AddRequirements(&m_vision);
}

// Called when the command is initially scheduled.
void RotateToAprilTarget::Initialize() {
  auto possibleTarget = m_vision.GetIdentifiedAprilTarget(m_ID);
  if (!possibleTarget.has_value()) {
    return;
  }
  auto target = possibleTarget.value();
}

// Called repeatedly when this Command is scheduled to run
void RotateToAprilTarget::Execute() {
  auto possibleTarget = m_vision.GetIdentifiedAprilTarget(m_ID);
  if (!possibleTarget.has_value()) {
    return;
  }
  auto target = possibleTarget.value();
  if (std::abs(int(m_drivebase.getPose().Rotation().Degrees()) % 180 -
               std::abs(target.GetYaw())) > 5) {
    m_drivebase.tankDrive(.1, -.1);
  } else {
    m_drivebase.tankDrive(-.1, .1);
  }
}

// Called once the command ends or is interrupted.
void RotateToAprilTarget::End(bool interrupted) {
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
  if (std::abs(std::abs(int(m_drivebase.getPose().Rotation().Degrees())) % 180 -
               std::abs(target.GetYaw())) > 5) {
    return true;
  }
  return false;
}
