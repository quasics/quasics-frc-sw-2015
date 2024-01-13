// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDrive.h"

ArcadeDrive::ArcadeDrive(Drivebase* drivebase,
                         std::function<double()> powerFunction,
                         std::function<double()> turnFunction)
    : m_drivebase(drivebase),
      m_powerFunction(powerFunction),
      m_turnFunction(turnFunction) {
  AddRequirements(drivebase);
}

// Called once the command ends or is interrupted.
void ArcadeDrive::UpdateSpeeds() {
  m_drivebase->ArcadeDrive(m_powerFunction(), m_turnFunction());
}
