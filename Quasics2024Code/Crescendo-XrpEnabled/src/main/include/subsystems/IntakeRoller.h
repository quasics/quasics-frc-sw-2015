// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeRoller : public frc2::SubsystemBase {
 public:
  IntakeRoller();

  void SetRollerSpeed(double percentSpeed);

  void Stop();

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_intake;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
