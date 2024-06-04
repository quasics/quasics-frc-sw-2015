// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class TransitionRoller : public frc2::SubsystemBase {
 public:
  TransitionRoller();

  void SetTransitionRollerSpeed(double percentSpeed);

  void Stop();

 private:
  rev::CANSparkMax m_transition;
};
