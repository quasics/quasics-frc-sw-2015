// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunConveyorAtSpeed.h"

RunConveyorAtSpeed::RunConveyorAtSpeed(Conveyor* conveyor, double speed)
    : m_conveyor(conveyor), m_conveyorSpeed(speed) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_conveyor);
}

// Called when the command is initially scheduled.
void RunConveyorAtSpeed::Initialize() {
  m_conveyor->SetConveyorSpeed(m_conveyorSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunConveyorAtSpeed::Execute() {
  m_conveyor->SetConveyorSpeed(m_conveyorSpeed);
}

// Called once the command ends or is interrupted.
void RunConveyorAtSpeed::End(bool interrupted) {
  m_conveyor->Stop();
}

// Returns true when the command should end.
bool RunConveyorAtSpeed::IsFinished() {
  return false;
}
