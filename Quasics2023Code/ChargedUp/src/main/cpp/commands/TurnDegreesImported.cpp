// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnDegreesImported.h"

#include <iostream>

TurnDegreesImported::TurnDegreesImported(Drivebase* drivebase, double speed,
                                         units::degree_t angle)
    : m_drivebase(drivebase),
      m_speed(angle > 0_deg ? std::abs(speed) : -std::abs(speed)),
      m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void TurnDegreesImported::Initialize() {
  angleTest = m_angle * DecreaseForCompensation;
  std::cout << "AngleTestValue" << angleTest.value() << std::endl;
  std::cout << "Wanted Turn" << m_angle.value() << std::endl;
  multiplier = 1.0;
  turningleft = angleTest > 0_deg;  // substitution
  // turningleft = m_angle > 0_deg;
  //  angleShift = m_drivebase->GetAngle();
  startingposition = m_drivebase->GetAngle();
  m_drivebase->SetBrakingMode(true);
  /*
  if (turningleft) {
    m_drivebase->TankDrive(-1 * m_speed, m_speed);
  } else {
    m_drivebase->TankDrive(-1 * m_speed, m_speed);
  }*/
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
}

// Called repeatedly when this Command is scheduled to run
void TurnDegreesImported::Execute() {
  m_drivebase->SetBrakingMode(true);
  if (turningleft) {
    // units::degree_t currentPosition = m_drivebase->GetAngle() - angleShift;
    units::degree_t currentPosition = m_drivebase->GetAngle();  // 200
    if (currentPosition >
            ((startingposition + angleTest) * 0.5) &&  // 200 > 200+90
        (m_speed * multiplier > 0.26)) {
      multiplier = multiplier * 0.95;
    }
    m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
  } else {
    units::degree_t currentPosition = m_drivebase->GetAngle();
    if (currentPosition < ((startingposition + angleTest) * 0.5) &&
        (m_speed * multiplier > 0.26)) {
      multiplier = multiplier * 0.95;
    }
    m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
  }
}

// Called once the command ends or is interrupted.
void TurnDegreesImported::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}
// Returns true when the command should end.
bool TurnDegreesImported::IsFinished() {
  units::degree_t currentPosition = m_drivebase->GetAngle();  // 200
  if (turningleft) {
    if (currentPosition >= (startingposition + angleTest)) {  // 200
      std::cout << "Difference Left "
                << (currentPosition - (startingposition + angleTest)).value()
                << std::endl;
      return true;
    } else {
      return false;
    }
  } else {
    if (currentPosition <= (startingposition + angleTest)) {
      std::cout << "Difference Right "
                << (currentPosition - (startingposition + angleTest)).value()
                << std::endl;
      return true;
    } else {
      return false;
    }
  }

  return false;
}
