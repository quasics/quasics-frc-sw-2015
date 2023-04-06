// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/DriveAtPowerForMetersWorkingVersion.h"

#include <iostream>

DriveAtPowerForMetersWorkingVersion::DriveAtPowerForMetersWorkingVersion(
    Drivebase* drivebase, double motorPower, units::meter_t distance)
    : m_drivebase(drivebase), m_motorPower(motorPower), m_distance(distance) {
  if (m_motorPower < 0 || m_distance < 0_m) {
    m_motorPower = -std::abs(m_motorPower);
    m_distance = -units::meter_t(std::abs(m_distance.value()));
  }
  AddRequirements(drivebase);
  SetName("DriveAtPowerForMetersWorkingVersion");
}

// Called when the command is initially scheduled.
void DriveAtPowerForMetersWorkingVersion::Initialize() {
  accelerating = true;
  subtraction = 0;
  gradualreduction = std::abs(m_motorPower) - 0.35;
  counter = 0;
  originalDistance = m_drivebase->GetLeftDistance();
  m_drivebase->ArcadeDrive(m_motorPower, 0);
  m_drivebase->SetBrakingMode(true);
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMetersWorkingVersion::Execute() {
  if (SlewRateLimiter.Calculate(m_motorPower) >= m_motorPower) {
    std::cout << "stopped accelerating" << std::endl;
    accelerating = false;
  }
  currentDistance = m_drivebase->GetLeftDistance();
  distanceToDestination = originalDistance + m_distance - currentDistance;
  if (std::abs(distanceToDestination.value()) < 1.5 &&
      std::abs(m_motorPower) > 0.35) {
    std::cout << "Applying Reduction" << std::endl;
    subtraction = std::abs(m_motorPower) - 0.35 - gradualreduction;
    counter++;
    if (gradualreduction > 0 && counter % 5 == 0) {
      if (gradualreduction <= 0) {
        gradualreduction = 0;
      } else {
        gradualreduction = gradualreduction - 0.1;
      }
    }
  }

  if (m_distance >= 0_m) {
    if (accelerating) {
      double motorValue = SlewRateLimiter.Calculate(m_motorPower);
      std::cout << "Sending Power to Motors" << motorValue << std::endl;
      m_drivebase->ArcadeDrive(motorValue, 0);
    } else {
      std::cout << "Sending Power to Motors" << m_motorPower - subtraction
                << std::endl;
      m_drivebase->ArcadeDrive(m_motorPower - subtraction, 0);
    }
  } else {
    if (accelerating) {
      double motorValue = SlewRateLimiter.Calculate(m_motorPower);
      std::cout << "Sending Power to Motors" << motorValue << std::endl;
      m_drivebase->ArcadeDrive(motorValue, -1 * 0);
    } else {
      std::cout << "Sending Power to Motors" << m_motorPower + subtraction
                << std::endl;
      m_drivebase->ArcadeDrive(m_motorPower + subtraction, -1 * 0);
    }
  }

  m_drivebase->SetBrakingMode(true);
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMetersWorkingVersion::End(bool interrupted) {
  std::cout << "finished" << std::endl;
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool DriveAtPowerForMetersWorkingVersion::IsFinished() {
  if (m_distance > 0_m) {
    if (currentDistance >= (originalDistance + m_distance)) {
      return true;
    }
  } else {
    if (currentDistance <= (originalDistance + m_distance)) {
      return true;
    }
  }
  return false;
}
