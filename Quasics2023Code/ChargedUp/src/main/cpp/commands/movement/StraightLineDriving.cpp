// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/StraightLineDriving.h"

#include <iostream>

// When button pressed, read current angle, in execute
// pid.calculate(currentAngle, to original reading) Use arcade drive give use
// forward power as what is given by joystick
// GetRawAxis(OperatorInterface::LogitechGamePad::LEFT_Y_AXIS)

// given distance: if speed negative distance becomes negative if speed positive
// distance becomes positive LATER now get starting distance if distance
// traveled is greater than starting distance stop!

StraightLineDriving::StraightLineDriving(Drivebase* drivebase, double speed,
                                         units::meter_t distance)
    : m_drivebase(drivebase),
      m_speed(speed),
      m_distance(speed > 0 ? distance : -distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
  SetName("StraightLineDriving");
}

// Called when the command is initially scheduled.
void StraightLineDriving::Initialize() {
  accelerating = true;
  subtraction = 0;
  gradualreduction = m_speed - 0.35;
  counter = 0;
  originalDistance = m_drivebase->GetLeftDistance();
  originalAngle = m_drivebase->GetYaw();
  std::cout << "Starting Angle: " << originalAngle.value() << std::endl;
  m_drivebase->ArcadeDrive(m_speed, 0);
  m_drivebase->SetBrakingMode(true);
}

// Called repeatedly when this Command is scheduled to run
void StraightLineDriving::Execute() {
  if (SlewRateLimiter.Calculate(m_speed) >= m_speed) {
    accelerating = false;
  }
  currentDistance = m_drivebase->GetLeftDistance();
  distanceToDestination = originalDistance + m_distance - currentDistance;
  currentAngle = m_drivebase->GetYaw();
  double rotationCorrection =
      pid.Calculate(currentAngle.value(), originalAngle.value());
  if (std::abs(distanceToDestination.value()) < 1.5 &&
      std::abs(m_speed) > 0.35) {
    // std::cout << "Applying Reduction" << std::endl;
    subtraction = std::abs(m_speed) - 0.35 - gradualreduction;
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
      m_drivebase->ArcadeDrive(SlewRateLimiter.Calculate(m_speed),
                               rotationCorrection);
    } else {
      m_drivebase->ArcadeDrive(m_speed - subtraction, rotationCorrection);
    }
  } else {
    if (accelerating) {
      m_drivebase->ArcadeDrive(SlewRateLimiter.Calculate(m_speed),
                               -1 * rotationCorrection);
    } else {
      m_drivebase->ArcadeDrive(m_speed + subtraction, -1 * rotationCorrection);
    }
  }

  m_drivebase->SetBrakingMode(true);
}

// Called once the command ends or is interrupted.
void StraightLineDriving::End(bool interrupted) {
  std::cout << "Ending Angle: " << currentAngle.value() << std::endl;
  m_drivebase->Stop();
  m_drivebase->SetBrakingMode(true);
}

// Returns true when the command should end.
bool StraightLineDriving::IsFinished() {
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
