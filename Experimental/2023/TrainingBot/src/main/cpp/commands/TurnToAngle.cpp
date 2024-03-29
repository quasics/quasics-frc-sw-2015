// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnToAngle.h"

#include <cmath>
#include <iostream>

TurnToAngle::TurnToAngle(DriveBase *driveBase, units::degree_t rotationDegree, double speed)
{
  m_driveBase = driveBase;
  m_angle = rotationDegree;
  m_speed = std::abs(speed);
  m_targetAngle = 0_deg;

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(driveBase);
}

// Called when the command is initially scheduled.
void TurnToAngle::Initialize()
{
  m_targetAngle = m_angle + m_driveBase->GetAngle();
  std::cerr << "Initialized Target angle: " << m_targetAngle.value() << "\n";
}

// Called repeatedly when this Command is scheduled to run
void TurnToAngle::Execute()
{
  const double SLOW_SPEED = 0.15;

   units::degree_t currentAngle = m_driveBase->GetAngle();
  if (m_angle.value() > 0)
  {
    if (currentAngle.value() >= m_targetAngle.value() - 5)
    {
      std::cerr << "Set speed to " << SLOW_SPEED << ", " << -SLOW_SPEED << "\n";
      m_driveBase->TankDrive(SLOW_SPEED, -SLOW_SPEED);
    } else {
      std::cerr << "Set speed to " << m_speed << ", " << -m_speed << "\n";
      m_driveBase->TankDrive(m_speed, -m_speed);
    }
  }
  else if (m_angle.value() < 0)
  {
    if (currentAngle.value() <= m_targetAngle.value() + 5)
    {
      std::cerr << "Set speed to " << -SLOW_SPEED << ", " << SLOW_SPEED << "\n";
      m_driveBase->TankDrive(-SLOW_SPEED, SLOW_SPEED);
    } else {
      std::cerr << "Set speed to " << -m_speed << ", " << m_speed << "\n";
      m_driveBase->TankDrive(-m_speed, m_speed);
    }
  }
}

// Called once the command ends or is interrupted.
void TurnToAngle::End(bool interrupted)
{
  std::cerr << "Stopping\n";
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished()
{
  units::degree_t currentAngle = m_driveBase->GetAngle();
  std::cerr << "current angle " << currentAngle.value() << ", target " << m_targetAngle.value() << "\n";
  if (m_angle == 0_deg)
  {
    std::cerr << "0 deg stop\n";
    return true;
  }
  else if (m_angle.value() > 0 && currentAngle >= (m_targetAngle - 0.5_deg))
  {
    std::cerr << ">0 angle stop\n";
    return true;
  }
  else if (m_angle.value() < 0 && currentAngle <= (m_targetAngle + 0.5_deg))
  {
    std::cerr << "<0 angle stop\n";
    return true;
  }
  return false;
}
