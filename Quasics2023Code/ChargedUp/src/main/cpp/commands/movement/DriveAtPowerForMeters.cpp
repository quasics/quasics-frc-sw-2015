// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/DriveAtPowerForMeters.h"

#include <iostream>

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase,
                                             double motorPower,
                                             units::meter_t distance)
    : m_drivebase(drivebase),
      m_motorPower(motorPower > 0 && distance > 0_m ? motorPower
                                                    : -std::abs(motorPower)),
      m_distance(motorPower > 0 && distance > 0_m
                     ? distance
                     : -units::meter_t(std::abs(distance.value()))) {
  AddRequirements(drivebase);
  SetName("DriveAtPowerForMeters");
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  m_accelerating = true;
  m_subtraction = 0;
  m_gradualreduction = std::abs(m_motorPower) - 0.35;
  m_counter = 0;
  m_originalDistance = m_drivebase->GetLeftDistance();
  m_drivebase->ArcadeDrive(m_motorPower, 0);
  m_drivebase->SetBrakingMode(true);
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  if (m_slewRateLimiter.Calculate(m_motorPower) >= m_motorPower) {
    std::cout << "stopped accelerating" << std::endl;
    m_accelerating = false;
  }
  m_currentDistance = m_drivebase->GetLeftDistance();
  m_distanceToDestination = m_originalDistance + m_distance - m_currentDistance;
  if (std::abs(m_distanceToDestination.value()) < 1.5 &&
      std::abs(m_motorPower) > 0.35) {
    std::cout << "Applying Reduction" << std::endl;
    m_subtraction = std::abs(m_motorPower) - 0.35 - m_gradualreduction;
    m_counter++;
    if (m_gradualreduction > 0 && m_counter % 5 == 0) {
      if (m_gradualreduction <= 0) {
        m_gradualreduction = 0;
      } else {
        m_gradualreduction = m_gradualreduction - 0.1;
      }
    }
  }

  if (m_distance >= 0_m) {
    if (m_accelerating) {
      double motorValue = m_slewRateLimiter.Calculate(m_motorPower);
      std::cout << "Sending Power to Motors" << motorValue << std::endl;
      m_drivebase->ArcadeDrive(motorValue, 0);
    } else {
      std::cout << "Sending Power to Motors" << m_motorPower - m_subtraction
                << std::endl;
      m_drivebase->ArcadeDrive(m_motorPower - m_subtraction, 0);
    }
  } else {
    if (m_accelerating) {
      double motorValue = m_slewRateLimiter.Calculate(m_motorPower);
      std::cout << "Sending Power to Motors" << motorValue << std::endl;
      m_drivebase->ArcadeDrive(motorValue, -1 * 0);
    } else {
      std::cout << "Sending Power to Motors" << m_motorPower + m_subtraction
                << std::endl;
      m_drivebase->ArcadeDrive(m_motorPower + m_subtraction, -1 * 0);
    }
  }

  m_drivebase->SetBrakingMode(true);
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  std::cout << "finished" << std::endl;
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  if (m_distance > 0_m) {
    if (m_currentDistance >= (m_originalDistance + m_distance)) {
      return true;
    }
  } else {
    if (m_currentDistance <= (m_originalDistance + m_distance)) {
      return true;
    }
  }
  return false;
}
