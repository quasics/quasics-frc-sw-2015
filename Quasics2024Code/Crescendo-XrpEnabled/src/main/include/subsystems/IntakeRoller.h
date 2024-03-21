// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

// TODO: Add docs for this class.
class IntakeRoller : public frc2::SubsystemBase {
 public:
  IntakeRoller();

  void SetRollerSpeed(double percentSpeed);

  void Stop();

 private:
  rev::CANSparkMax m_intake;
};
