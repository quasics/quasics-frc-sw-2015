// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoFloorRetract.h"

#include <iostream>

AutoFloorRetract::AutoFloorRetract(FloorEjection* floorEjection,
                                   double retractionSpeed)
    : m_floorEjection(floorEjection),
      m_retractionSpeed(-std::abs(retractionSpeed)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(floorEjection);
}

// Called when the command is initially scheduled.
void AutoFloorRetract::Initialize() {
  std::cout << "Limit Switch Closed" << m_floorEjection->FloorRetracted()
            << std::endl;
  m_floorEjection->SetFloorEjectionPower(m_retractionSpeed);
}

// Called repeatedly when this Command is scheduled to run
void AutoFloorRetract::Execute() {
  m_floorEjection->SetFloorEjectionPower(m_retractionSpeed);
}

// Called once the command ends or is interrupted.
void AutoFloorRetract::End(bool interrupted) {
  std::cout << "Stopping Flipper" << std::endl;
  m_floorEjection->Stop();
}

// Returns true when the command should end.
bool AutoFloorRetract::IsFinished() {
  if (m_floorEjection->FloorRetracted()) {
    std::cerr << "Limit switch says it's pressed.\n";
    counter++;
    counterReset = 0;
    if (counter > 5) {
      return true;
    }
  }
  std::cerr << "Limit switch says it's not pressed.\n";
  if (counterReset > 1) {
    counter = 0;
  }
  counterReset++;
  return false;
}
