// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/TurnDegreesImported.h"

#include <iostream>

TurnDegreesImported::TurnDegreesImported(Drivebase* drivebase, double speed,
                                         units::degree_t angle)
    : m_drivebase(drivebase),
      m_speed(angle > 0_deg ? std::abs(speed) : -std::abs(speed)),
      m_angle(angle) {
  AddRequirements(m_drivebase);
  SetName("TurnDegreesImported");
}

// Called when the command is initially scheduled.
void TurnDegreesImported::Initialize() {
  // if angle is greater than 0 and turning left no problem, if angle less than
  // 0 and turning right no problem, if angle is greater than 0 turning right
  // and less than 90 problem if angle is less than 0 and turning left and
  // greater than -90 problem
  //
  // CODE_REVIEW(matthew): What "problem"?  And what needs to be done about it?
  m_subtraction = 0;
  m_startingposition = m_drivebase->GetYaw();
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
}

// Called repeatedly when this Command is scheduled to run
// CODE_REVIEW(matthew): Lots of "magic #s" in here, which should be cleaned up
// (and documented as to how they were detemined, what they do, etc.).
void TurnDegreesImported::Execute() {
  units::degree_t currentPosition = m_drivebase->GetYaw();
  if (std::abs((m_startingposition + m_angle - currentPosition).value()) <
          52.5 &&
      (std::abs(m_speed) > 0.3)) {
    m_subtraction = std::abs(m_speed) - 0.3;
    std::cout << "Invoking subtraction" << std::endl;
  }
  m_drivebase->SetBrakingMode(true);
  if (m_speed > 0) {
    m_drivebase->TankDrive(-1 * (m_speed - m_subtraction),
                           (m_speed - m_subtraction));
  } else {
    m_drivebase->TankDrive(-1 * (m_speed + m_subtraction),
                           (m_speed + m_subtraction));
  }
}

// Called once the command ends or is interrupted.
void TurnDegreesImported::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}
// Returns true when the command should end.
// CODE_REVIEW(matthew): Lots of "magic #s" in here, which should be cleaned up
// (and documented as to how they were detemined, what they do, etc.).
bool TurnDegreesImported::IsFinished() {
  if (std::abs(m_angle.value()) == 90) {
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (m_angle > 0_deg) {
      if (currentPosition > m_startingposition + m_angle - 12_deg) {  // was 35
        return true;
      }
    } else {
      if (currentPosition < m_startingposition + m_angle + 12_deg) {
        return true;
      }
    }
    return false;
  } else {
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (m_angle > 0_deg) {
      if (currentPosition > m_startingposition + m_angle - 20_deg) {
        return true;
      }
    } else {
      if (currentPosition < m_startingposition + m_angle + 20_deg) {
        return true;
      }
    }
    return false;
  }
  return false;
}
