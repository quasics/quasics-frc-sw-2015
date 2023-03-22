// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnDegreesImported.h"

#include <iostream>

#define CURRENT_TURN_VERSION
#undef FASTER_SLOW_DOWN
#undef NO_SLOW_DOWN
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
#ifdef CURRENT_TURN_VERSION
  angleTest = m_angle * DecreaseForCompensation;
  std::cout << "AngleTestValue" << angleTest.value() << std::endl;
  std::cout << "Wanted Turn" << m_angle.value() << std::endl;
  multiplier = 1.0;
  turningleft = angleTest > 0_deg;  // substitution
  startingposition = m_drivebase->GetYaw();
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
#endif
#ifdef NO_SLOW_DOWN
  startingposition = m_drivebase->GetYaw();
  turningleft = m_angle > 0_deg;
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
#endif
}

// Called repeatedly when this Command is scheduled to run
void TurnDegreesImported::Execute() {
#ifdef CURRENT_TURN_VERSION
  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetYaw();
  if (turningleft) {
    if (currentPosition > ((startingposition + angleTest) * 0.5) &&
        (m_speed * multiplier > 0.30)) {
#ifdef FASTER_SLOW_DOWN
      multiplier = multiplier * 0.90;
#endif
      multiplier = multiplier * 0.95;
    }
    m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
  } else {
    if (currentPosition < ((startingposition + angleTest) * 0.5) &&
        (m_speed * multiplier > 0.30)) {
#ifdef FASTER_SLOW_DOWN
      multiplier = multiplier * 0.90;
#endif
      multiplier = multiplier * 0.95;
    }
    m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
  }
#endif
#ifdef NO_SLOW_DOWN
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
#endif
}

// Called once the command ends or is interrupted.
void TurnDegreesImported::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}
// Returns true when the command should end.
bool TurnDegreesImported::IsFinished() {
#ifdef CURRENT_TURN_VERSION
  units::degree_t currentPosition = m_drivebase->GetYaw();
  if (turningleft) {
    if (currentPosition >= (startingposition + angleTest)) {
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
#endif

#ifdef NO_SLOW_DOWN
  units::degree_t currentPosition = m_drivebase->GetAngle();
  if (turningleft) {
    if (currentPosition >=
        (startingposition + m_angle) -
            15_deg) {  // arbitrary value to have time to stop turning
      return true;
    }
    return false;
  } else {
    if (currentPosition <=
        (startingposition + m_angle) +
            15_deg) {  // arbitrary value to have time to stop turning
      return true;
    }
    return false;
  }
#endif
}
