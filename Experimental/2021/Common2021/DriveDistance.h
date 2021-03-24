// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include <cmath>

#include "../../../../Common2021/CommonDriveSubsystem.h"
#include "../../../../Common2021/MathUtils.h"

/**
 * Drives for the specified distance, at a rated power.
 */
class DriveDistance
    : public frc2::CommandHelper<frc2::CommandBase, DriveDistance> {
 public:
  /**
   * Constructor.
   *
   * @param drive    the drive subsystem to be used
   * @param distance the distance to be traveled, relative to the starting point
   *                 (i.e., 2m == 2 meters forward, -4.5m == 4.5 meters
   *                 backward)
   * @param power    the power to be applied to the wheels (will be capped to
   *                 [-1..+1], and sign-corrected to match the distance)
   */
  DriveDistance(CommonDriveSubsystem* drive, units::meter_t distance,
                double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  CommonDriveSubsystem* m_drive;
  const units::meter_t m_distanceToTravel;
  const double m_power;
  units::meter_t m_distanceWhenFinished;
};

inline DriveDistance::DriveDistance(CommonDriveSubsystem* drive,
                                    units::meter_t distance, double power)
    : m_drive(drive),
      m_distanceToTravel(distance),
      // Power will always be corrected to the direction we need to go (positive
      // for forward, negative for backward), and capped at 100%.
      m_power(sign(distance.to<double>()) * std::min(1.0, std::abs(power))),
      m_distanceWhenFinished(units::meter_t(0)) {
  AddRequirements(m_drive);
}

// Called when the command is initially scheduled.
inline void DriveDistance::Initialize() {
  m_distanceWhenFinished = m_drive->GetAverageDistance() + m_distanceToTravel;
}

// Called repeatedly when this Command is scheduled to run
inline void DriveDistance::Execute() {
  m_drive->TankDrive(m_power, m_power);
}

// Called once the command ends or is interrupted.
inline void DriveDistance::End(bool interrupted) {
  m_drive->Stop();
}

// Returns true when the command should end.
inline bool DriveDistance::IsFinished() {
  if (m_power > 0) {
    // We're driving forward
    return m_drive->GetAverageDistance() >= m_distanceWhenFinished;
  } else {
    // We're driving backward
    return m_drive->GetAverageDistance() <= m_distanceWhenFinished;
  }
}
