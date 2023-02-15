// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveUntilPitchAngleChange.h"

DriveUntilPitchAngleChange::DriveUntilPitchAngleChange(Drivebase *drivebase,
                                                       double power,
                                                       units::meter_t distance)
    : m_drivebase(drivebase),
      m_power((power < 0 || distance.value() < 0) ? -std::abs(power)
                                                  : std::abs(power)),
      m_distance(std::abs(distance.value())) {
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveUntilPitchAngleChange::Initialize() {
  m_drivebase->TankDrive(m_power, m_power);
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitchAngleChange::Execute() {
  m_drivebase->TankDrive(m_power, m_power);
}

// Called once the command ends or is interrupted.
void DriveUntilPitchAngleChange::End(bool interrupted) { m_drivebase->Stop(); }

// Returns true when the command should end.
bool DriveUntilPitchAngleChange::IsFinished() {
  double currentAngle = m_drivebase->GetPitch();
  units::meter_t distanceLeft = m_drivebase->GetLeftDistance();
  units::meter_t distanceRight = m_drivebase->GetRightDistance();

  // NOT DONE CONTINUE WORKING ON THIS
  if (currentAngle > 10 || currentAngle < -10) {
    return true;
  } else if (m_power >= 0) {
    return (distanceLeft >= (m_distance + m_leftStartingPosition) ||
            distanceRight >= (m_distance + m_rightStartingPosition));
  } else {
    return (distanceLeft <= (m_leftStartingPosition - m_distance) ||
            distanceRight <= (m_rightStartingPosition - m_distance));
  }
  return false;
}
