// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Null.h"

Null::Null(Drivetrain* driveTrain, units::meter_t distance, double speedAsPercent)
 : m_driveTrain(driveTrain), m_distance(distance), m_speedAsPercent(speedAsPercent) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({driveTrain});
}

// Called when the command is initially scheduled.
void Null::Initialize() {
  // m_driveTrain->ResetGyro();
  // m_driveTrain->ResetEncoders();
  // m_driveTrain->TankDrive(m_speedAsPercent, (m_speedAsPercent * 0.75));
}

// Called repeatedly when this Command is scheduled to run
void Null::Execute() {}

// Called once the command ends or is interrupted.
void Null::End(bool interrupted) {
    m_driveTrain->TankDrive(0,0);
}

// Returns true when the command should end.
bool Null::IsFinished() {
  if(m_driveTrain->GetGyroAngleZ() >= 360) {
     return true;
   }
  return false;
}
