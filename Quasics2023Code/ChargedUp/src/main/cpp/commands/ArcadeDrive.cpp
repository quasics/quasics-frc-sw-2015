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
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void ArcadeDrive::Initialize() {
  UpdateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
  UpdateSpeeds();
}

// Called once the command ends or is interrupted.
void ArcadeDrive::End(bool interrupted) {
  m_drivebase->Stop();
}

void ArcadeDrive::UpdateSpeeds() {
  m_drivebase->ArcadeDrive(m_powerFunction(), m_turnFunction());
}
