// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunConveyorAtSpeed.h"

RunConveyorAtSpeed::RunConveyorAtSpeed(Conveyor* conveyor, double speed)
    : m_Conveyor(conveyor), m_ConveyorSpeed(speed) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_Conveyor);
}

// Called when the command is initially scheduled.
void RunConveyorAtSpeed::Initialize() {
  m_Conveyor->SetConveyorSpeed(m_ConveyorSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunConveyorAtSpeed::Execute() {
  m_Conveyor->SetConveyorSpeed(m_ConveyorSpeed);
}

// Called once the command ends or is interrupted.
void RunConveyorAtSpeed::End(bool interrupted) {
  m_Conveyor->Stop();
}

// Returns true when the command should end.
bool RunConveyorAtSpeed::IsFinished() {
  return false;
}
